# Qorvo UWB → rover pose

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional, Dict, Tuple

import numpy as np
import serial 
from scipy.optimize import least_squares  

log = logging.getLogger(__name__)

# Geometry / system constants 
ORDER = ["A", "B", "C", "D"]

DX_TAGS = 0.61   # x separation between rover tags (m) (Real value 0.61)
DY_TAGS = 0.64   # y separation between rover tags (m) (Real value 0.64
DZ_TAGS = 0.33   # tag height above ground (m) (Real value 0.61)
THETA_OFFSET = math.atan2(DY_TAGS, DX_TAGS)

max_skew_threshold = 0.25  # max time skew between tag 1 and 2 (s)
max_age_threshold = 0.5    # max age of a tag measurement (s)

# Anchor ID → letter
ANCHOR_MAP: Dict[str, str] = {
    "0A91": "A",
    "D593": "B",
    "16A7": "C",
    "CF33": "D",
}

# Anchor positions in arena frame (meters)
ANCHOR_POS: Dict[str, Tuple[float, float, float]] = {
    "A": (2.0, 0.0, 0.42),
    "B": (1.0, 0.0, 1.02),
    "C": (0.0, 1.0, 0.42),
    "D": (0.0, 2.0, 1.02),
}

@dataclass
class Pose:
    x: float   # m, arena frame
    y: float   # m
    theta: float  # rad, yaw
    cov_xx: float = 0.0
    cov_yy: float = 0.0
    cov_tt: float = 0.0
    timestamp: float = 0.0


class UWBDriver:
    """Abstract interface for UWB-based localization."""

    def get_pose(self) -> Optional[Pose]:
        """Return current rover pose in arena frame, or None if unavailable."""
        raise NotImplementedError

    def close(self) -> None:
        pass


class MockUWB(UWBDriver):
    """Always returns a fixed pose. Use for bench testing without UWB hardware."""

    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self._pose = Pose(x=x, y=y, theta=theta, timestamp=time.time())

    def get_pose(self) -> Optional[Pose]:
        self._pose.timestamp = time.time()
        return self._pose


# ---------------------------------------------------------------------
# Qorvo UWB implementation (2 tags, multilateration + yaw)
# ---------------------------------------------------------------------


class QorvoUWB(UWBDriver):
    """
    Reads two Qorvo tags over serial (e.g. 'Beacon_1', 'Beacon_2'),
    trilaterates each tag w.r.t. 4 anchors, then combines them into
    rover (x, y, yaw) in arena frame.
    """

    def __init__(
        self,
        ports: Dict[str, str],
        baud: int = 115200,
    ) -> None:
        """
        ports: mapping like {"Beacon_1": "COM12", "Beacon_2": "COM13"}
        """
        self._ports_cfg = ports
        self._baud = baud

        # A1..D1, A2..D2 distances
        self._values: Dict[str, Optional[float]] = {
            "A1": None, "B1": None, "C1": None, "D1": None,
            "A2": None, "B2": None, "C2": None, "D2": None,
        }

        # Last multilateration results for each tag
        self._last_data = {
            "1": {"pos": None, "time": 0.0},
            "2": {"pos": None, "time": 0.0},
        }

        self._pose: Optional[Pose] = None
        self._lock = threading.Lock()
        self._running = True
        self._serials: Dict[str, serial.Serial] = {}

        self._start_hardware()

    # ------------------- hardware / threads -------------------

    def _start_hardware(self) -> None:
        # Open and init each beacon/tag
        for name, port in self._ports_cfg.items():
            ser = self._init_beacon(name, port)
            if ser:
                self._serials[name] = ser

        # Start a listener thread per beacon
        for name, ser in self._serials.items():
            t = threading.Thread(
                target=self._listen_to_beacon,
                args=(name, ser),
                daemon=True,
            )
            t.start()

        # Start a small background thread to run multilateration + pose fusion
        threading.Thread(target=self._solver_loop, daemon=True).start()

    def _init_beacon(self, name: str, port: str) -> Optional[serial.Serial]:
        try:
            ser = serial.Serial(port, self._baud, timeout=1)
            time.sleep(1.0)
            # Wake / initialize
            ser.write(b"\r")
            time.sleep(0.5)
            ser.write(b"\r")
            time.sleep(0.5)
            # Start logging
            ser.write(b"les\r")
            time.sleep(1.0)
            # Optional: read and log response
            _ = ser.read_all().decode("utf-8", errors="ignore")
            log.info("UWB %s initialized on %s", name, port)
            return ser
        except serial.SerialException as e:
            log.error("Error opening UWB %s on %s: %s", name, port, e)
            return None

    def _listen_to_beacon(self, name: str, ser: serial.Serial) -> None:
        tag = "1" if name.endswith("1") else "2"
        while self._running:
            try:
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    self._parse_beacon_line(tag, line)
            except Exception:
                # swallow errors, continue trying
                time.sleep(0.01)

    # ------------------- parsing / math -------------------

    def _parse_beacon_line(self, tag: str, line: str) -> None:
        """
        Parse one raw Qorvo line, update A<tag>..D<tag> distances.
        Expected tokens like '0A91=2.34' (anchor_id=distance_m).
        """
        parts = line.split()
        with self._lock:
            for part in parts:
                try:
                    anchor_id = part[:4]
                    distance = float(part.split("=")[1])
                    anchor_letter = ANCHOR_MAP.get(anchor_id)
                    if anchor_letter:
                        self._values[f"{anchor_letter}{tag}"] = distance
                except (IndexError, ValueError):
                    continue

    def _distances_ready(self, tag: str) -> bool:
        keys = [f"A{tag}", f"B{tag}", f"C{tag}", f"D{tag}"]
        return all(self._values[k] is not None for k in keys)

    def _get_beacon_distances(self, tag: str) -> Dict[str, float]:
        return {
            "A": self._values[f"A{tag}"],
            "B": self._values[f"B{tag}"],
            "C": self._values[f"C{tag}"],
            "D": self._values[f"D{tag}"],
        }

    def _is_synced(self, t1: float, t2: float) -> bool:
        now = time.time()
        return (
            abs(t1 - t2) < max_skew_threshold
            and (now - t1) < max_age_threshold
            and (now - t2) < max_age_threshold
        )

    def _multilaterate_2p5d(self, distances: Dict[str, float], last_data: Dict) -> Tuple[float, float, float]:
        def residuals(xy):
            x, y = xy
            res = []
            for a in ORDER:
                if a not in distances or distances[a] is None:
                    continue
                ax, ay, az = ANCHOR_POS[a]
                d = distances[a]
                pred = math.sqrt(
                    (x - ax) ** 2 +
                    (y - ay) ** 2 +
                    (DZ_TAGS - az) ** 2
                )
                res.append(pred - d)
            return res

        if last_data["pos"] is None:
            xs = [ANCHOR_POS[a][0] for a in ORDER]
            ys = [ANCHOR_POS[a][1] for a in ORDER]
            x0 = [sum(xs) / len(xs), sum(ys) / len(ys)]
        else:
            x0 = list(last_data["pos"][:2])

        sol = least_squares(
            residuals,
            x0=x0,
            bounds=([0, 0], [6.88, 5.0]),
        )
        return sol.x[0], sol.x[1], DZ_TAGS

    def _rover_pose_from_tags(self, pos1, pos2) -> Tuple[np.ndarray, float]:
        p1 = np.array(pos1)
        p2 = np.array(pos2)
        center = (p1 + p2) / 2.0
        measured_angle = math.atan2(
            p1[1] - p2[1],  # Tag2 → Tag1 vector
            p1[0] - p2[0],
        )
        yaw = measured_angle - THETA_OFFSET     # rad
        # Normalize to [-pi, pi]
        yaw = (yaw + math.pi) % (2.0 * math.pi) - math.pi
        return center, yaw

    def _solver_loop(self) -> None:
        while self._running:
            now = time.time()
            with self._lock:
                pos1 = pos2 = None

                if self._distances_ready("1"):
                    d1 = self._get_beacon_distances("1")
                    pos1 = self._multilaterate_2p5d(d1, self._last_data["1"])
                    self._last_data["1"] = {"pos": pos1, "time": now}
                    for k in ["A1", "B1", "C1", "D1"]:
                        self._values[k] = None

                if self._distances_ready("2"):
                    d2 = self._get_beacon_distances("2")
                    pos2 = self._multilaterate_2p5d(d2, self._last_data["2"])
                    self._last_data["2"] = {"pos": pos2, "time": now}
                    for k in ["A2", "B2", "C2", "D2"]:
                        self._values[k] = None

                p1_data = self._last_data["1"]
                p2_data = self._last_data["2"]

                if p1_data["pos"] is not None and p2_data["pos"] is not None:
                    if self._is_synced(p1_data["time"], p2_data["time"]):
                        center, yaw = self._rover_pose_from_tags(p1_data["pos"], p2_data["pos"])
                        self._pose = Pose(
                            x=float(center[0]),
                            y=float(center[1]),
                            theta=float(yaw),
                            timestamp=now,
                        )

            time.sleep(0.05)  # ~20 Hz

    # ------------------- public API -------------------

    def get_pose(self) -> Optional[Pose]:
        with self._lock:
            return self._pose

    def close(self) -> None:
        self._running = False
        for ser in self._serials.values():
            try:
                if ser and ser.is_open:
                    ser.close()
            except Exception:
                pass

def create_uwb(mock: bool = True, **kwargs) -> UWBDriver:
    if mock:
        return MockUWB(**kwargs)
    # Expect kwargs["ports"] and optionally kwargs["baud"]
    ports = kwargs.get("ports")
    if not ports:
        raise ValueError("QorvoUWB requires 'ports' mapping, e.g. {'Beacon_1': 'COM12', 'Beacon_2': 'COM13'}")
    baud = kwargs.get("baud", 115200)
    return QorvoUWB(ports=ports, baud=baud)