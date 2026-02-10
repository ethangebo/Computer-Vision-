# occupancy grid
import numpy as np
import cv2

def _logodds(p: float) -> float:
    p = float(np.clip(p, 1e-6, 1 - 1e-6))
    return np.log(p / (1.0 - p))

class OccupancyGrid2D:
    """
    2D occupancy grid in rover frame:
      - x axis: forward (meters)
      - y axis: left (meters)

    Stored as log-odds in self.L, shape (H, W):
      row increases downward, col increases rightward.
     map:
      col ~ x forward
      row ~ y left (with sign handled)
    """
    def __init__(self, width_m, height_m, res_m, p_occ=0.7, p_free=0.3, l_min=-3.0, l_max=3.0):
        self.res = float(res_m)
        self.width_m = float(width_m)
        self.height_m = float(height_m)

        self.w = int(np.ceil(self.width_m / self.res))   # cols
        self.h = int(np.ceil(self.height_m / self.res))  # rows

        # Center the grid on the rover for y (left/right), but for x we prefer "center" too
        # place rover at (x=0,y=0) at grid center.
        self.cx = self.w // 2
        self.cy = self.h // 2

        self.L = np.zeros((self.h, self.w), dtype=np.float32)

        self.L_occ = _logodds(p_occ)
        self.L_free = _logodds(p_free)
        self.l_min = float(l_min)
        self.l_max = float(l_max)
    
    def decay(self, amount=0.02):
        """
        Slowly decay log-odds toward unknown.
        Call once per frame.
        """
        self.L *= (1.0 - amount)

    def reset(self):
        self.L.fill(0.0)

    def world_to_grid(self, x_fwd_m, y_left_m):
        # x -> col, y -> row
        c = int(np.floor(x_fwd_m / self.res)) + self.cx
        r = self.cy - int(np.floor(y_left_m / self.res))
        return r, c

    def grid_to_world(self, r, c):
        """Cell (r, c) -> (x_forward, y_left) meters, cell center. Rover frame."""
        x_fwd_m = (c - self.cx + 0.5) * self.res
        y_left_m = (self.cy - r - 0.5) * self.res
        return x_fwd_m, y_left_m

    def in_bounds(self, r, c):
        return 0 <= r < self.h and 0 <= c < self.w

    def _bresenham(self, r0, c0, r1, c1):
        # integer grid line cells
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dc - dr

        r, c = r0, c0
        while True:
            yield r, c
            if r == r1 and c == c1:
                break
            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                c += sc
            if e2 < dc:
                err += dc
                r += sr

    def _ray_cells(self, r0, c0, r1, c1):
        """Return (rr_free, cc_free), (rr_occ, cc_occ) for ray. Occ = last cell only."""
        cells = list(self._bresenham(r0, c0, r1, c1))
        if len(cells) < 2:
            return None, None
        rr = np.array([c[0] for c in cells], dtype=np.int32)
        cc = np.array([c[1] for c in cells], dtype=np.int32)
        free = (rr[:-1], cc[:-1])
        occ = (rr[-1:], cc[-1:])
        return free, occ

    def update_rays(self, endpoints_xy, sensor_origin_xy=(0.0, 0.0)):
        """
        endpoints_xy: (N,2) array in (x_forward, y_left) meters.
        Raycast from sensor_origin to each endpoint:
          cells along ray -> free
          endpoint cell -> occupied
        Vectorized: batch all rays, then np.add.at + clip.
        """
        sox, soy = sensor_origin_xy
        r0, c0 = self.world_to_grid(sox, soy)
        h, w = self.h, self.w

        free_r, free_c = [], []
        occ_r, occ_c = [], []

        for i in range(endpoints_xy.shape[0]):
            x, y = float(endpoints_xy[i, 0]), float(endpoints_xy[i, 1])
            r1, c1 = self.world_to_grid(x, y)
            if not (0 <= r1 < h and 0 <= c1 < w):
                continue
            free, occ = self._ray_cells(r0, c0, r1, c1)
            if free is None:
                continue
            free_r.append(free[0])
            free_c.append(free[1])
            occ_r.append(occ[0])
            occ_c.append(occ[1])

        if not free_r:
            return

        free_r = np.concatenate(free_r)
        free_c = np.concatenate(free_c)
        occ_r = np.concatenate(occ_r)
        occ_c = np.concatenate(occ_c)

        keep = (free_r >= 0) & (free_r < h) & (free_c >= 0) & (free_c < w)
        free_r, free_c = free_r[keep], free_c[keep]
        keep = (occ_r >= 0) & (occ_r < h) & (occ_c >= 0) & (occ_c < w)
        occ_r, occ_c = occ_r[keep], occ_c[keep]

        np.add.at(self.L, (free_r, free_c), self.L_free)
        np.add.at(self.L, (occ_r, occ_c), self.L_occ)
        np.clip(self.L, self.l_min, self.l_max, out=self.L)

    def mark_occupied_cells(self, xy):
        """
        Mark (x_forward, y_left) footprint cells as occupied. No raycast.
        Use for craters (negative obstacles): project floor points to 2D, mark footprint.
        """
        if xy is None or xy.shape[0] == 0:
            return
        h, w = self.h, self.w
        rr = []
        cc = []
        for i in range(xy.shape[0]):
            r, c = self.world_to_grid(float(xy[i, 0]), float(xy[i, 1]))
            if 0 <= r < h and 0 <= c < w:
                rr.append(r)
                cc.append(c)
        if not rr:
            return
        rr = np.array(rr, dtype=np.int32)
        cc = np.array(cc, dtype=np.int32)
        np.add.at(self.L, (rr, cc), self.L_occ)
        np.clip(self.L, self.l_min, self.l_max, out=self.L)

    def get_prob(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.L))

    def get_prob_inflated(self, radius_m, occ_threshold=0.65):
        """
        Return P with inflated obstacle zones set to 1.0. Does not mutate L.
        Use for planning so free-space evidence is preserved in stored log-odds.
        """
        P = self.get_prob()
        occ = (P >= occ_threshold).astype(np.uint8) * 255
        rad_px = max(1, int(np.ceil(radius_m / self.res)))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * rad_px + 1, 2 * rad_px + 1))
        occ_dil = cv2.dilate(occ, kernel)
        out = P.copy()
        out[occ_dil > 0] = 1.0
        return out

    def inflate(self, radius_m, occ_threshold=0.65):
        """Legacy: mutate L with inflated obstacles. Prefer get_prob_inflated for planning."""
        P = self.get_prob()
        occ = (P >= occ_threshold).astype(np.uint8) * 255
        rad_px = max(1, int(np.ceil(radius_m / self.res)))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*rad_px+1, 2*rad_px+1))
        occ_dil = cv2.dilate(occ, kernel)
        self.L[occ_dil > 0] = self.l_max

    def to_debug_image(self):
        """
        Grayscale:
          unknown ~ 127
          free darker
          occupied brighter
        """
        P = self.get_prob()
        img = (P * 255.0).astype(np.uint8)
        return img
