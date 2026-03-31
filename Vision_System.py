import threading
import time
import numpy as np
import config as C
from realsense_io import RealSenseDepth
from occupancy_grid import OccupancyGrid2D
from global_grid import GlobalOccupancyGrid2D, stamp_local_into_global
from mapping import Mapper

class VisionSystem(threading.Thread):
    def __init__(self, global_grid):
        super().__init__(daemon=True)
        self.global_grid = global_grid
        
        # Front camera (always used when moving forward)
        self.front_cam = RealSenseDepth(
            depth_res=C.DEPTH_RES, fps=C.FPS, z_max_m=C.Z_MAX_M
        )
        
        # Rear camera (started at boot, used only when reversing)
        self.rear_cam = None
        if C.REAR_CAMERA_SERIAL is not None:
            try:
                self.rear_cam = RealSenseDepth(
                    depth_res=C.DEPTH_RES, fps=C.FPS, z_max_m=C.Z_MAX_M,
                    serial=C.REAR_CAMERA_SERIAL
                )
            except Exception:
                self.rear_cam = None
        
        self.local_grid = OccupancyGrid2D(
            width_m=C.GRID_WIDTH_M, height_m=C.GRID_HEIGHT_M, res_m=C.GRID_RES_M,
            p_occ=C.P_OCC, p_free=C.P_FREE, l_min=C.L_MIN, l_max=C.L_MAX
        )
        
        # Mapper handles Boulder/Crater identification and coordinate transforms
        self.mapper = Mapper(
            grid=self.local_grid, point_stride=C.POINT_STRIDE,
            z_min=C.Z_MIN_M, z_max=C.Z_MAX_M
        )
        
        self.current_pose = None
        self.running = True
        
        self._reversing = False
        self._reversing_lock = threading.Lock()

    def update_reversing(self, reversing: bool):
        """Call from main loop when target_v < 0. Switches to rear camera for obstacle mapping."""
        with self._reversing_lock:
            self._reversing = reversing

    def update_pose(self, x, y, theta_rad):
        """Used by the main loop to give Vision the UWB/Fused pose for global stamping."""
        from collections import namedtuple
        Pose = namedtuple('Pose', ['x', 'y', 'theta'])
        self.current_pose = Pose(x, y, theta_rad)

    def run(self):
        while self.running:
            with self._reversing_lock:
                use_rear = (self._reversing or C.DEBUG_FORCE_REAR_CAMERA) and self.rear_cam is not None
                
            cam = self.rear_cam if use_rear else self.front_cam
            depth_frame, depth_m = cam.wait_depth(timeout_ms=6000)
            
            if depth_m is not None:
                # 1. Decay local grid so old obstacles fade
                self.local_grid.decay(amount=0.02)
                
                # 2. Extract boulder and crater coordinates (camera_facing for rear coord transform)
                camera_facing = "rear" if use_rear else "front"
                endpoints_xy, crater_xy, _ = self.mapper.update_from_depth(
                    depth_m, cam.fx, cam.fy, cam.cx, cam.cy,
                    camera_facing=camera_facing
                )
                
                # 3. Integrate into local grid
                self.mapper.integrate(
                    endpoints_xy, crater_xy=crater_xy,
                    sensor_origin_xy=(0.0, 0.0),
                    inflate_radius_m=C.ROVER_RADIUS_M,
                    inflate_occ_th=C.INFLATE_OCC_THRESHOLD
                )

                # 4. Stamp local data into Global Map for A* rerouting
                if self.current_pose is not None:
                    stamp_local_into_global(
                        self.local_grid, self.global_grid, 
                        self.current_pose, occ_threshold=C.INFLATE_OCC_THRESHOLD
                    )
            time.sleep(0.001)

    def stop_vision(self):
        self.running = False
        self.front_cam.stop()
        if self.rear_cam is not None:
            self.rear_cam.stop()