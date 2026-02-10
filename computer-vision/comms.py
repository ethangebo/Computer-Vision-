# comms.py — Send drive commands to myRIO (Lunabotics)
#
# Interface: send(steer, speed) or send_velocity(v_left, v_right).
# Replace MockMyRIO with real TCP/serial implementation when hardware is ready.
# Ensure myRIO firmware uses the same protocol (format, units, rate).

from __future__ import annotations

import logging
import time
from typing import Optional

log = logging.getLogger(__name__)


class MyRIOComms:
    """Abstract interface for sending drive commands to myRIO."""

    def send(self, steer: float, speed: float) -> bool:
        """
        Send steering and speed to myRIO.
        steer: typically in [-1, 1] (left/right)
        speed: typically in [0, 1] (stop to max)
        Returns True if sent successfully.
        """
        raise NotImplementedError

    def estop(self) -> bool:
        """Emergency stop: send speed=0 (and optionally dedicated E-stop message)."""
        return self.send(0.0, 0.0)

    def close(self) -> None:
        """Release connection."""
        pass


class MockMyRIO(MyRIOComms):
    """Logs commands only. Use for bench testing without myRIO."""

    def __init__(self, rate_hz: float = 10.0):
        self.rate_hz = rate_hz
        self._last_send = 0.0

    def send(self, steer: float, speed: float) -> bool:
        now = time.time()
        if now - self._last_send >= 1.0 / self.rate_hz:
            log.debug("myRIO (mock): steer=%.3f speed=%.3f", steer, speed)
            self._last_send = now
        return True


# -----------------------------------------------------------------------------
# Real myRIO implementation (placeholder)
# -----------------------------------------------------------------------------
# Typical options:
#   - TCP socket to myRIO IP (e.g. 192.168.1.2) and port
#   - Serial (USB-UART) if you use a serial bridge
#   - LabVIEW shared variable / NI-VISA if you use NI stack
#
# Example TCP sketch (adapt to your myRIO program):
#
#   import socket
#   class TCPMyRIO(MyRIOComms):
#       def __init__(self, host: str = "192.168.1.2", port: int = 5555):
#           self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#           self.sock.connect((host, port))
#       def send(self, steer: float, speed: float) -> bool:
#           msg = f"{steer:.3f},{speed:.3f}\n"   # or binary protocol
#           self.sock.sendall(msg.encode())
#           return True
#       def close(self) -> None:
#           self.sock.close()
# -----------------------------------------------------------------------------


def create_comms(mock: bool = True, **kwargs) -> MyRIOComms:
    """Create comms instance. Use mock=True until myRIO hardware is wired."""
    if mock:
        return MockMyRIO(**kwargs)
    # return TCPMyRIO(**kwargs)  # or SerialMyRIO, etc.
    raise NotImplementedError("Real myRIO implementation not yet added")


# Optional: rate limit commands sent to myRIO (e.g. 10–20 Hz)
def rate_limit_send(comms: MyRIOComms, steer: float, speed: float, min_interval_s: float = 0.1) -> bool:
    """Throttle send calls to min_interval_s."""
    now = time.time()
    if not hasattr(rate_limit_send, "_last"):
        rate_limit_send._last = 0.0
    if now - rate_limit_send._last < min_interval_s:
        return True
    rate_limit_send._last = now
    return comms.send(steer, speed)
