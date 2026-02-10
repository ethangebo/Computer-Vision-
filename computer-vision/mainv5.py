import os
os.environ["PYTHONWARNINGS"] = "ignore"

import cv2
import numpy as np
import time

import config as C
from hud import toast, draw_toast
from realsense_io import RealSenseDepth
from occupancy_grid import OccupancyGrid2D
from global_grid import GlobalOccupancyGrid2D, stamp_local_into_global
from localization.uwb import create_uwb
from mapping import Mapper
from planner import plan_from_grid
from viz import depth_to_colormap, draw_status, draw_boxes_on_depth, grid_to_color, grid_to_color_global


def _hstack_match_height(img_a, img_b, interp=cv2.INTER_NEAREST):
    """
    Resize img_b to match img_a height (preserving aspect ratio), then hconcat.
    Prevents "smushed" views when panels have different heights.
    """
    ha = img_a.shape[0]
    hb = img_b.shape[0]
    if hb != ha:
        scale = ha / float(hb)
        new_w = max(1, int(img_b.shape[1] * scale))
        img_b = cv2.resize(img_b, (new_w, ha), interpolation=interp)
    return cv2.hconcat([img_a, img_b])


def main():
    cam = RealSenseDepth(depth_res=C.DEPTH_RES, fps=C.FPS, z_max_m=C.Z_MAX_M)

    grid = OccupancyGrid2D(
        width_m=C.GRID_WIDTH_M,
        height_m=C.GRID_HEIGHT_M,
        res_m=C.GRID_RES_M,
        p_occ=C.P_OCC,
        p_free=C.P_FREE,
        l_min=C.L_MIN,
        l_max=C.L_MAX
    )

    global_grid = GlobalOccupancyGrid2D(
        width_m=getattr(C, "GLOBAL_GRID_WIDTH_M", 20.0),
        height_m=getattr(C, "GLOBAL_GRID_HEIGHT_M", 10.0),
        res_m=getattr(C, "GLOBAL_GRID_RES_M", 0.10),
        p_occ=C.P_OCC,
        p_free=C.P_FREE,
        l_min=C.L_MIN,
        l_max=C.L_MAX,
        decay_amount=getattr(C, "GLOBAL_DECAY_AMOUNT", 0.0),
    )
    uwb = create_uwb(
        mock=getattr(C, "UWB_MOCK", True),
        ports=getattr(C, "UWB_PORTS", {}),
        baud=getattr(C, "UWB_BAUD", 115200),
    )
    
    mapper = Mapper(
        grid=grid,
        point_stride=C.POINT_STRIDE,
        z_min=C.Z_MIN_M,
        z_max=C.Z_MAX_M,
        use_roi=C.USE_ROI,
        roi_top=C.ROI_TOP,
        roi_side=C.ROI_SIDE,
        plane_fit_rows_frac=C.PLANE_FIT_ROWS_FRAC,
        plane_sample_max=C.PLANE_SAMPLE_MAX,
        plane_ransac_iters=C.PLANE_RANSAC_ITERS,
        plane_inlier_tol=C.PLANE_INLIER_TOL_M,
        plane_refit_every=C.PLANE_REFIT_EVERY,
        ground_band_m=C.GROUND_BAND_M,
        min_obstacle_height_m=C.MIN_OBSTACLE_HEIGHT_M,
        max_obstacle_height_m=C.MAX_OBSTACLE_HEIGHT_M,
        obstacle_min_cluster_size=C.OBSTACLE_MIN_CLUSTER_SIZE,
        obstacle_downsample_res_m=C.OBSTACLE_DOWNSAMPLE_RES_M,
        crater_height_below_m=C.CRATER_HEIGHT_BELOW_M,
        max_crater_depth_m=C.MAX_CRATER_DEPTH_M,
        filter_walls=getattr(C, "FILTER_WALLS", True),
        wall_footprint_max_m=getattr(C, "WALL_FOOTPRINT_MAX_M", 0.70),
        wall_merge_dist_m=getattr(C, "WALL_MERGE_DIST_M", 0.12),
        wall_max_points_per_cluster=getattr(C, "WALL_MAX_POINTS_PER_CLUSTER", 50),
    )

    cv2.namedWindow(C.WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(C.WINDOW_NAME, 1600, 900)

    last_t = time.time()
    fps_smooth = 0.0

    try:
        while True:
            depth_frame, depth_m = cam.wait_depth(timeout_ms=6000)
            if depth_m is None:
                continue

            # 1) Decay first, then depth -> endpoints (boulders + craters) -> integrate
            grid.decay(amount=0.02)
            endpoints_xy, crater_xy, stats = mapper.update_from_depth(depth_m, cam.fx, cam.fy, cam.cx, cam.cy)
            mapper.integrate(
                endpoints_xy,
                crater_xy=crater_xy,
                sensor_origin_xy=(0.0, 0.0),
                inflate_radius_m=C.ROVER_RADIUS_M,
                inflate_occ_th=C.INFLATE_OCC_THRESHOLD,
            )

            # 2) Stamp local occupied cells into global map (UWB pose)
            pose = uwb.get_pose()
            if pose is not None:
                stamp_local_into_global(grid, global_grid, pose, occ_threshold=0.65)

            # 3) Plan from grid (uses inflated view for planning)
            steer, speed, dbg = plan_from_grid(
                grid,
                sector_degs=C.SECTOR_DEGS,
                lookahead_m=C.LOOKAHEAD_M,
                corridor_half_width_m=C.CORRIDOR_HALF_WIDTH_M,
                stop_dist_m=C.STOP_DIST_M,
                slow_dist_m=C.SLOW_DIST_M,
                inflate_radius_m=C.ROVER_RADIUS_M,
                inflate_occ_th=C.INFLATE_OCC_THRESHOLD,
            )

            # 4) Visualization panels
            panels = []

            # Depth view (+ object boxes for testing)
            if C.SHOW_DEPTH_VIEW:
                vis_depth = depth_to_colormap(depth_m, C.Z_MAX_M)
                P = grid.get_prob()
                occ_cells = int((P > 0.65).sum())
                nc = stats.get("n_craters", 0)
                draw_status(vis_depth, steer, speed, text2=f"occ:{occ_cells} craters:{nc} hdg:{dbg.get('heading')}")
                if C.SHOW_BOXES:
                    draw_boxes_on_depth(
                        vis_depth,
                        stats.get("bboxes_boulders_uv", []),
                        stats.get("bboxes_craters_uv", []),
                        thickness=2,
                    )
                draw_toast(vis_depth)
                panels.append(vis_depth)

            # Grid view (local costmap)
            if C.SHOW_GRID_VIEW:
                grid_img = grid_to_color(
                    grid,
                    scale=8,
                    heading_deg=dbg.get("heading"),
                )
                panels.append(grid_img)

            # Global map (arena frame, persistent)
            if C.SHOW_GLOBAL_GRID:
                pose = uwb.get_pose()
                pose_xy = (pose.x, pose.y) if pose else None
                global_img = grid_to_color_global(global_grid, pose_xy=pose_xy, scale=4)
                panels.append(global_img)

            if panels:
                out = panels[0]
                for p in panels[1:]:
                    out = _hstack_match_height(out, p, interp=cv2.INTER_NEAREST)

                # FPS overlay
                now = time.time()
                dt = now - last_t
                last_t = now
                inst_fps = (1.0 / dt) if dt > 1e-6 else 0.0
                fps_smooth = 0.9 * fps_smooth + 0.1 * inst_fps
                cv2.putText(out, f"{fps_smooth:.1f} fps", (10, out.shape[0]-15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

                cv2.imshow(C.WINDOW_NAME, out)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
            elif key == ord('r'):
                grid.reset()
                toast("Local grid reset")
            elif key == ord('R'):
                global_grid.reset()
                toast("Global map reset")
            elif key in (ord('v'), ord('V')):
                C.SHOW_DEPTH_VIEW = not C.SHOW_DEPTH_VIEW
                toast(f"Depth view {'ON' if C.SHOW_DEPTH_VIEW else 'OFF'}")
            elif key in (ord('g'), ord('G')):
                C.SHOW_GRID_VIEW = not C.SHOW_GRID_VIEW
                toast(f"Grid view {'ON' if C.SHOW_GRID_VIEW else 'OFF'}")
            elif key in (ord('m'), ord('M')):
                C.SHOW_GLOBAL_GRID = not C.SHOW_GLOBAL_GRID
                toast(f"Global map {'ON' if C.SHOW_GLOBAL_GRID else 'OFF'}")
            elif key in (ord('b'), ord('B')):
                C.SHOW_BOXES = not C.SHOW_BOXES
                toast(f"Object boxes {'ON' if C.SHOW_BOXES else 'OFF'}")

    finally:
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
