# planner.py
import numpy as np
import math

def _grid_sample_points(grid, heading_deg, lookahead_m, half_width_m, n_steps=30, n_width=7):
    """
    Sample points in a corridor for a given heading.
    Returns list of (x_forward, y_left) points to evaluate.
    """
    th = math.radians(heading_deg)
    # unit direction in rover frame (x_forward, y_left)
    dx = math.cos(th)
    dy = math.sin(th)

    xs = np.linspace(0.2, lookahead_m, n_steps)
    ws = np.linspace(-half_width_m, half_width_m, n_width)

    pts = []
    for x in xs:
        # centerline point
        cx = x * dx
        cy = x * dy
        # offsets sideways relative to heading
        # sideways unit vector:
        sx = -dy
        sy = dx
        for w in ws:
            pts.append((cx + w*sx, cy + w*sy))
    return pts

def plan_from_grid(
    grid,
    sector_degs,
    lookahead_m,
    corridor_half_width_m,
    stop_dist_m,
    slow_dist_m,
    inflate_radius_m=None,
    inflate_occ_th=0.65,
):
    """
    Returns (steer_cmd, speed_cmd, debug)
      steer_cmd in [-1..1] approx (scaled from chosen heading)
      speed_cmd in [0..1]
    If inflate_radius_m is set, uses get_prob_inflated for planning.
    """
    if inflate_radius_m is not None and inflate_radius_m > 0:
        P = grid.get_prob_inflated(radius_m=inflate_radius_m, occ_threshold=inflate_occ_th)
    else:
        P = grid.get_prob()
    res = grid.res

    # emergency stop check: look directly forward in a narrow corridor
    forward_pts = _grid_sample_points(grid, 0.0, stop_dist_m, half_width_m=0.20, n_steps=20, n_width=5)
    for (xf, yl) in forward_pts:
        r, c = grid.world_to_grid(xf, yl)
        if grid.in_bounds(r, c) and P[r, c] > 0.65:
            return 0.0, 0.0, {"reason": "stop", "heading": 0.0}

    # score each candidate heading by occupancy probability in corridor
    best = None
    scores = {}
    for hdg in sector_degs:
        pts = _grid_sample_points(grid, hdg, lookahead_m, corridor_half_width_m)
        s = 0.0
        n = 0
        for (xf, yl) in pts:
            r, c = grid.world_to_grid(xf, yl)
            if not grid.in_bounds(r, c):
                continue
            s += float(P[r, c])
            n += 1
        score = s / max(n, 1)
        scores[hdg] = score

        if best is None or score < best[0]:
            best = (score, hdg)

    _, chosen = best

    # speed control: if forward corridor (slow_dist) has occupancy, slow down
    slow_pts = _grid_sample_points(grid, 0.0, slow_dist_m, half_width_m=0.25, n_steps=25, n_width=5)
    slow_score = 0.0
    slow_n = 0
    for (xf, yl) in slow_pts:
        r, c = grid.world_to_grid(xf, yl)
        if grid.in_bounds(r, c):
            slow_score += float(P[r, c])
            slow_n += 1
    slow_avg = slow_score / max(slow_n, 1)

    if slow_avg > 0.25:
        speed = 0.3
    else:
        speed = 0.7

    # steering: normalize chosen heading roughly to [-1..1]
    max_abs = max(abs(float(h)) for h in sector_degs) if sector_degs else 35.0
    steer = float(chosen) / float(max_abs)

    return steer, speed, {"reason": "grid", "heading": chosen, "scores": scores, "slow_avg": slow_avg}
