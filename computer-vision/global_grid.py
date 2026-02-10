# global_grid.py — Persistent arena map in UWB/arena frame
#
# GlobalOccupancyGrid2D: fixed in arena frame, no (or slow) decay.
# stamp_local_into_global: copy local occupied cells into global map using UWB pose.

from __future__ import annotations

import math
import numpy as np

from occupancy_grid import _logodds


class GlobalOccupancyGrid2D:
    """
    2D occupancy grid in arena frame (UWB / field coordinates).
    - X axis: arena forward (e.g. along lane)
    - Y axis: arena left
    Origin at grid center. No decay by default (persistent map).
    """

    def __init__(
        self,
        width_m: float,
        height_m: float,
        res_m: float,
        p_occ: float = 0.7,
        p_free: float = 0.3,
        l_min: float = -3.0,
        l_max: float = 3.0,
        decay_amount: float = 0.0,
    ):
        self.res = float(res_m)
        self.width_m = float(width_m)
        self.height_m = float(height_m)
        self.w = int(np.ceil(self.width_m / self.res))
        self.h = int(np.ceil(self.height_m / self.res))
        self.cx = self.w // 2
        self.cy = self.h // 2
        self.L = np.zeros((self.h, self.w), dtype=np.float32)
        self.L_occ = _logodds(p_occ)
        self.L_free = _logodds(p_free)
        self.l_min = float(l_min)
        self.l_max = float(l_max)
        self.decay_amount = float(decay_amount)

    def decay(self, amount: float | None = None) -> None:
        """Optional slow decay. Default uses self.decay_amount."""
        a = amount if amount is not None else self.decay_amount
        if a <= 0:
            return
        self.L *= 1.0 - a

    def reset(self) -> None:
        self.L.fill(0.0)

    def world_to_grid(self, x_m: float, y_m: float) -> tuple[int, int]:
        c = int(np.floor(x_m / self.res)) + self.cx
        r = self.cy - int(np.floor(y_m / self.res))
        return r, c

    def in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self.h and 0 <= c < self.w

    def mark_occupied(self, x_m: float, y_m: float) -> None:
        r, c = self.world_to_grid(x_m, y_m)
        self.add_occupancy_at(r, c)

    def add_occupancy_at(self, r: int, c: int) -> bool:
        """Add L_occ at cell (r, c). Returns True if in bounds."""
        if not self.in_bounds(r, c):
            return False
        self.L[r, c] = np.clip(self.L[r, c] + self.L_occ, self.l_min, self.l_max)
        return True

    def get_prob(self) -> np.ndarray:
        return 1.0 - 1.0 / (1.0 + np.exp(self.L))


def _rover_to_global(x_r: float, y_r: float, x: float, y: float, theta: float) -> tuple[float, float]:
    """Transform (x_r, y_r) in rover frame (forward, left) -> (x_g, y_g) in arena frame."""
    c = math.cos(theta)
    s = math.sin(theta)
    x_g = x + c * x_r - s * y_r
    y_g = y + s * x_r + c * y_r
    return x_g, y_g


def stamp_local_into_global(
    local_grid,
    global_grid: GlobalOccupancyGrid2D,
    pose,  # Pose with .x, .y, .theta (rad)
    occ_threshold: float = 0.65,
) -> int:
    """
    Copy local occupied cells into global map using UWB pose.
    Returns number of cells stamped.
    """
    P = local_grid.get_prob()
    occ = P > occ_threshold
    rr, cc = np.where(occ)
    x, y = pose.x, pose.y
    theta = pose.theta
    n = 0
    for r, c in zip(rr, cc):
        x_r, y_r = local_grid.grid_to_world(r, c)
        x_g, y_g = _rover_to_global(x_r, y_r, x, y, theta)
        r_g, c_g = global_grid.world_to_grid(x_g, y_g)
        if global_grid.add_occupancy_at(r_g, c_g):
            n += 1
    return n
