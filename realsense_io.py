# realsense_io.py
import pyrealsense2 as rs
import numpy as np

class RealSenseDepth:
    """
    Depth-only RealSense wrapper:
    - wait_depth() -> (depth_frame, depth_m)
    - intrinsics stored as fx, fy, cx, cy
    Coordinate convention of deprojection:
      X: lateral (right+ in image coords; depending on camera)
      Y: vertical (down+ in image coords)
      Z: forward distance
    """
    def __init__(self, depth_res=(848,480), fps=30, z_max_m=6.0, serial=None):
        """
        serial: Optional device serial string. Use to select a specific camera when multiple are connected.
        """
        rs.log_to_console(rs.log_severity.none)

        self.pipeline = rs.pipeline()
        cfg = rs.config()
        if serial is not None:
            cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.depth, depth_res[0], depth_res[1], rs.format.z16, fps)
        self.profile = self.pipeline.start(cfg)

        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        intr = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        self.fx, self.fy, self.cx, self.cy = intr.fx, intr.fy, intr.ppx, intr.ppy

        # Filters (keep it simple + stable)
        self.decimate = rs.decimation_filter(2)
        self.spatial  = rs.spatial_filter()
        self.temporal = rs.temporal_filter()
        self.threshold = rs.threshold_filter(min_dist=0.10, max_dist=float(z_max_m))

    def stop(self):
        self.pipeline.stop()

    def wait_depth(self, timeout_ms=6000):
        frames = self.pipeline.wait_for_frames(timeout_ms)
        d = frames.get_depth_frame()
        if not d:
            return None, None

        d = self.decimate.process(d)
        d = self.spatial.process(d)
        d = self.temporal.process(d)
        d = self.threshold.process(d)

        depth_raw = np.asanyarray(d.get_data(), dtype=np.uint16)
        depth_m   = depth_raw.astype(np.float32) * self.depth_scale
        return d, depth_m

    def deproject_pixels(self, u, v, z_m):
        # Standard pinhole deprojection from depth intrinsics
        X = (u - self.cx) * z_m / self.fx
        Y = (v - self.cy) * z_m / self.fy
        Z = z_m
        return X, Y, Z
