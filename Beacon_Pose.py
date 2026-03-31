import serial
import time
import threading
import numpy as np
import math
from scipy.optimize import least_squares
import config as C

class BeaconPose:

    def start(self):
        self.running = True
        for name, port in self.ports.items():
            ser = self.init_beacon(name, port)
            if ser:
                self.serial_devices[name] = ser
                t = threading.Thread(
                    target=self.listen_to_beacon,
                    args=(name, ser),
                    daemon=True
                )
                t.start()
    
    # Stops beacon threads and closes serial ports.
    def stop(self):
       
        # Tell threads to exit
        self.running = False

        # Close all serial ports
        for ser in self.serial_devices.values():
            try:
                if ser and ser.is_open:
                    ser.close()
            except Exception:
                pass

        self.serial_devices.clear()
        
    def __init__(self, ports={"Beacon_1": "COM12", "Beacon_2": "COM13"}):
        # Configuration and Constants
        self.ORDER = ["A", "B", "C", "D"]
        self.DX_TAGS = C.UWB_DX_TAGS_M
        self.DY_TAGS = C.UWB_DY_TAGS_M
        self.DZ_TAGS = C.UWB_DZ_TAGS_M
        self.THETA_OFFSET = math.atan2(self.DY_TAGS, self.DX_TAGS)
        self.MAX_SKEW = C.UWB_MAX_SKEW_S
        self.MAX_AGE = C.UWB_MAX_AGE_S
        self.BAUD_RATE = C.UWB_BAUD_RATE
        self.ANCHOR_POS = C.UWB_ANCHOR_POS
        self.ANCHOR_MAP = C.UWB_ANCHOR_MAP

        # State Variables
        self.lock = threading.Lock()

        self.values = {
            "A1": None, "B1": None, "C1": None, "D1": None,
            "A2": None, "B2": None, "C2": None, "D2": None,
        }
        self.last_data = {
            "1": {"pos": None, "time": 0},
            "2": {"pos": None, "time": 0}
        }
        
        self.serial_devices = {}
        self.ports = ports
        self.running = False

    # Initialize beacon and return serial object
    def init_beacon(self, name, port):

        try:
            ser = serial.Serial(port, self.BAUD_RATE, timeout=1)
            # print(f"[{name}] Connected on {port}")

            time.sleep(1)

            # Wake / initialize
            ser.write(b'\r')
            time.sleep(0.5)
            ser.write(b'\r')
            time.sleep(0.5)

            # Start logging
            ser.write(b'les\r')
            time.sleep(1)

            response = ser.read_all().decode("utf-8", errors="ignore") #ignore error
            # print(f"[{name}] Init response:\n{response}")

            return ser

        except serial.SerialException as e:
            # print(f"[{name}] ERROR opening {port}: {e}")
            return None
    # Continuously read data from a beacon       
    def listen_to_beacon(self, name, ser):

        # print(f"[{name}] Listening")
        try:
            while self.running:
                # Read a single line from the serial buffer
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    # Attach the beacon name so the parser knows the source
                    tagged_line = f"[{name}] {line}"
                    # print(tagged_line) # Useful for debugging
                    self.parse_beacon_line(tagged_line)

        except Exception:
            # If the serial port is closed or disconnected, the thread exits
            pass

    def parse_beacon_line(self, line):
        if "[Beacon_1]" in line:
            tag = "1"
        elif "[Beacon_2]" in line:
            tag = "2"
        else:
            return

        try:
            # Splits the line to isolate the anchor data parts
            parts = line.split("] ", 1)[1].split()
        except IndexError:
            return

        for part in parts:
            try:
                # Extracts the 4-character ID and the distance after the '='
                anchor_id = part[:4]
                distance = float(part.split("=")[1])

                anchor_letter = self.ANCHOR_MAP.get(anchor_id)
                if anchor_letter:
                    # Updates the central storage dictionary
                    with self.lock:
                        self.values[f"{anchor_letter}{tag}"] = distance # ignore error

            except (IndexError, ValueError):
                continue 
    def get_beacon_distances(self, tag):
        """
        tag: "1" or "2"
        returns a dictionary {A,B,C,D: distance}
        """
        return {
            "A": self.values[f"A{tag}"],
            "B": self.values[f"B{tag}"],
            "C": self.values[f"C{tag}"],
            "D": self.values[f"D{tag}"],
        }

    def distances_ready(self, tag):
        """Returns True if all 4 anchor distances are available for this tag"""
        keys = [f"A{tag}", f"B{tag}", f"C{tag}", f"D{tag}"]
        # Checks if every required key is no longer None
        return all(self.values[k] is not None for k in keys)

    def is_synced(self, t1, t2):
        """
        Ensure neither reading is too old and they are close to each other.
        max_skew_threshold and max_age_threshold are defined in __init__.
        """
        now = time.time()
        # Returns True if skew and age are within limits
        return (abs(t1 - t2) < self.MAX_SKEW) and \
               (now - t1 < self.MAX_AGE) and \
               (now - t2 < self.MAX_AGE)   
    
    def multilaterate_2p5d(self, distances, tag_id):
        """
        Solves for the (x, y) position of a tag using anchor distances.
        tag_id: "1" or "2"
        """
        def residuals(xy):
            x, y = xy
            res = []
            for a in self.ORDER:
                if a not in distances:
                    continue
                ax, ay, az = self.ANCHOR_POS[a]
                d = distances[a]
                # Calculate distance from guess to anchor
                pred = math.sqrt(
                    (x - ax)**2 +
                    (y - ay)**2 +
                    (self.DZ_TAGS - az)**2
                )
                res.append(pred - d)
            return res

        # Determine initial guess for the optimizer
        last_pos = self.last_data[tag_id]["pos"]
        if last_pos is None:
            xs = [self.ANCHOR_POS[a][0] for a in self.ORDER]
            ys = [self.ANCHOR_POS[a][1] for a in self.ORDER]
            x0 = [sum(xs)/len(xs), sum(ys)/len(ys)]
        else:
            x0 = list(last_pos[:2])

        # Solve for the best x, y fit
        sol = least_squares(
            residuals,
            x0=x0,
            bounds=([0, 0], [C.GLOBAL_GRID_WIDTH_M, C.GLOBAL_GRID_HEIGHT_M])
        )

        return sol.x[0], sol.x[1], self.DZ_TAGS
    
    def rover_pose(self, pos1, pos2):
        p1 = np.array(pos1)
        p2 = np.array(pos2)

        center_beacon = (p1 + p2) / 2

        measured_angle = math.atan2(
            p1[1] - p2[1],  # Tag2 → Tag1 vector
            p1[0] - p2[0]
        )

        yaw_beacon = math.degrees(measured_angle) - math.degrees(self.THETA_OFFSET)

        return center_beacon, yaw_beacon
    
    def get_update(self):
        """
        Calculates the latest available pose thread-safely.
        Returns (center, yaw) or (None, None) if not ready or out of sync.
        """
        d1 = None
        d2 = None
        
        # 1. Safely extract Tag 1 distances
        with self.lock:
            if self.distances_ready("1"):
                d1 = self.get_beacon_distances("1")
                # Reset values immediately so we wait for fresh data
                for k in ["A1", "B1", "C1", "D1"]: self.values[k] = None
                
        # Calculate Tag 1 outside the lock to prevent blocking serial reads
        if d1:
            pos1 = self.multilaterate_2p5d(d1, "1")
            with self.lock:
                self.last_data["1"] = {"pos": pos1, "time": time.time()}

        # 2. Safely extract Tag 2 distances
        with self.lock:
            if self.distances_ready("2"):
                d2 = self.get_beacon_distances("2")
                for k in ["A2", "B2", "C2", "D2"]: self.values[k] = None

        # Calculate Tag 2 outside the lock
        if d2:
            pos2 = self.multilaterate_2p5d(d2, "2")
            with self.lock:
                self.last_data["2"] = {"pos": pos2, "time": time.time()}

        # 3. Check synchronization safely 
        with self.lock:
            # Snapshot the dicts to avoid a race condition during sync check
            ld1 = self.last_data["1"].copy()
            ld2 = self.last_data["2"].copy()

        if ld1["pos"] and ld2["pos"]:
            if self.is_synced(ld1["time"], ld2["time"]): 
                return self.rover_pose(ld1["pos"], ld2["pos"])
                
        return None, None