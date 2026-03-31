import math
import config as C

class ComplementaryFusion:
    def __init__(self):
        """
        alpha_pos : position correction gain (0-1)
        alpha_yaw : yaw correction gain (0-1)
        """
        
        # Config Parameters
        self.alpha_pos = C.ALPHA_POS
        self.alpha_yaw = C.ALPHA_YAW

        self.center_fused = [0.0, 0.0, 0.0]
        self.yaw_fused = 0.0

        # Track previous odometry to calculate deltas
        self.last_odom_center = None
        self.last_odom_yaw = None

        self.initialized = False

    # Angle-safe difference 
    @staticmethod
    def angle_diff_deg(a, b):
        """Returns smallest signed difference a - b (degrees)."""
        return (a - b + 180.0) % 360.0 - 180.0

    # Main update 
    def update(self, odom_center, odom_yaw, beacon_center, beacon_yaw):
        # 0. Initialize
        if not self.initialized:
            self.center_fused = list(odom_center)
            self.yaw_fused = odom_yaw
            self.last_odom_center = list(odom_center)
            self.last_odom_yaw = odom_yaw
            self.initialized = True
            return self.center_fused, self.yaw_fused

        # 1. Prediction Step (Add Odometry Deltas)
        # How much did odometry say rover moved since the last frame?
        dx_odom = odom_center[0] - self.last_odom_center[0]
        dy_odom = odom_center[1] - self.last_odom_center[1]
        dyaw_odom = self.angle_diff_deg(odom_yaw, self.last_odom_yaw)

        # Apply that movement to fused state
        self.center_fused[0] += dx_odom
        self.center_fused[1] += dy_odom
        self.yaw_fused = (self.yaw_fused + dyaw_odom) % 360.0

        # Save current odometry for the next frame's delta calculation
        self.last_odom_center = list(odom_center)
        self.last_odom_yaw = odom_yaw

        # 2. Update Step (Apply Beacon Correction)
        if beacon_center is not None and beacon_yaw is not None:
            # How far off is fused state from absolute beacon reading?
            dx_err = beacon_center[0] - self.center_fused[0]
            dy_err = beacon_center[1] - self.center_fused[1]

            # Gently pull fused state toward the beacon
            self.center_fused[0] += self.alpha_pos * dx_err
            self.center_fused[1] += self.alpha_pos * dy_err

            yaw_err = self.angle_diff_deg(beacon_yaw, self.yaw_fused)
            self.yaw_fused = (self.yaw_fused + self.alpha_yaw * yaw_err) % 360.0

        return self.center_fused, self.yaw_fused