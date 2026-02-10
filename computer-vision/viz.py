# viz.py
import numpy as np
import cv2

def depth_to_colormap(depth_m, z_max_m):
    d = np.clip(depth_m, 0, z_max_m)
    norm = (255.0 * d / (z_max_m if z_max_m > 0 else 1.0)).astype(np.uint8)
    return cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)

def draw_status(img, steer, speed, text2=None):
    cv2.putText(img, f"steer={steer:+.2f}  speed={speed:.2f}", (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
    if text2:
        cv2.putText(img, text2, (10, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)

def _draw_boxes_on_grid(img, grid, boxes_xyxy, color_bgr, thickness=1):
    """Draw 2D boxes (x_min,y_min,x_max,y_max) in rover frame on grid image (H,W,3)."""
    H, W = img.shape[:2]
    for (x_min, y_min, x_max, y_max) in (boxes_xyxy or []):
        corners = [(x_min, y_min), (x_min, y_max), (x_max, y_min), (x_max, y_max)]
        rs, cs = [], []
        for x, y in corners:
            r, c = grid.world_to_grid(x, y)
            rs.append(r)
            cs.append(c)
        c_lo = max(0, min(cs))
        c_hi = min(W - 1, max(cs))
        r_lo = max(0, min(rs))
        r_hi = min(H - 1, max(rs))
        if c_hi < c_lo or r_hi < r_lo:
            continue
        cv2.rectangle(img, (c_lo, r_lo), (c_hi, r_hi), color_bgr, thickness)


def draw_boxes_on_depth(img, boxes_uv_boulders, boxes_uv_craters, thickness=2):
    """
    Draw 2D boxes on depth image (BGR). boxes_*: list of (u_min, v_min, u_max, v_max) pixel coords.
    Boulders green, craters cyan.
    """
    H, W = img.shape[:2]
    for (u_min, v_min, u_max, v_max) in (boxes_uv_boulders or []):
        x0 = int(max(0, min(u_min, u_max)))
        x1 = int(min(W - 1, max(u_min, u_max)))
        y0 = int(max(0, min(v_min, v_max)))
        y1 = int(min(H - 1, max(v_min, v_max)))
        if x1 <= x0 or y1 <= y0:
            continue
        cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), thickness)
    for (u_min, v_min, u_max, v_max) in (boxes_uv_craters or []):
        x0 = int(max(0, min(u_min, u_max)))
        x1 = int(min(W - 1, max(u_min, u_max)))
        y0 = int(max(0, min(v_min, v_max)))
        y1 = int(min(H - 1, max(v_min, v_max)))
        if x1 <= x0 or y1 <= y0:
            continue
        cv2.rectangle(img, (x0, y0), (x1, y1), (255, 255, 0), thickness)


def grid_to_color(grid, scale=8, heading_deg=None, boxes_boulders=None, boxes_craters=None):
    """
    Color-coded occupancy grid (local):
      black   = free
      gray    = unknown
      red     = occupied
      green   = rover origin
      blue    = heading arrow (optional)
    boxes_boulders / boxes_craters: list of (x_min,y_min,x_max,y_max) in rover frame (optional).
    """
    P = grid.get_prob()
    H, W = P.shape

    img = np.zeros((H, W, 3), dtype=np.uint8)

    free = (P < 0.35)
    unk  = (P >= 0.35) & (P <= 0.65)
    occ  = (P > 0.65)

    img[free] = (0, 0, 0)
    img[unk]  = (120, 120, 120)
    img[occ]  = (0, 0, 255)

    # rover origin
    r0, c0 = grid.cy, grid.cx
    img[r0, c0] = (0, 255, 0)

    # optional heading arrow
    if heading_deg is not None:
        import math
        th = math.radians(float(heading_deg))
        length_cells = int(1.5 / grid.res)  # ~1.5m arrow
        r1 = int(r0 - length_cells * math.sin(th))
        c1 = int(c0 + length_cells * math.cos(th))
        if 0 <= r1 < H and 0 <= c1 < W:
            cv2.arrowedLine(img, (c0, r0), (c1, r1), (255, 0, 0), 1, tipLength=0.25)

    # optional object boxes on grid (boulders green, craters cyan)
    _draw_boxes_on_grid(img, grid, boxes_boulders, (0, 255, 0), 1)
    _draw_boxes_on_grid(img, grid, boxes_craters, (255, 255, 0), 1)

    # scale up for viewing
    img_big = cv2.resize(img, (W * scale, H * scale), interpolation=cv2.INTER_NEAREST)
    return img_big


def grid_to_color_global(grid, pose_xy=None, scale=4):
    """
    Global arena map: same color scheme, optional rover marker at pose_xy (x_g, y_g) meters.
    """
    P = grid.get_prob()
    H, W = P.shape

    img = np.zeros((H, W, 3), dtype=np.uint8)
    free = (P < 0.35)
    unk  = (P >= 0.35) & (P <= 0.65)
    occ  = (P > 0.65)
    img[free] = (0, 0, 0)
    img[unk]  = (120, 120, 120)
    img[occ]  = (0, 0, 255)

    if pose_xy is not None:
        x_g, y_g = pose_xy[0], pose_xy[1]
        r, c = grid.world_to_grid(x_g, y_g)
        if 0 <= r < H and 0 <= c < W:
            cv2.circle(img, (c, r), max(1, int(0.15 / grid.res)), (0, 255, 0), -1)

    img_big = cv2.resize(img, (W * scale, H * scale), interpolation=cv2.INTER_NEAREST)
    return img_big
