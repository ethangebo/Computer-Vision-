import math
import config as C

class ConstrainedPurePursuit:
    def __init__(self):
        # Config Parameters
        self.Ld = C.LOCAL_LOOKAHEAD_M
        self.min_radius = C.MIN_TURNING_RADIUS_M
        self.target_speed = C.TARGET_SPEED_M_S
        
        # Calculate the maximum allowable curvature based on turning radius
        self.max_curvature = 1.0 / self.min_radius

    def _get_lookahead_point(self, x, y, path):
        """
        Finds the target point on the path that is exactly 'Ld' distance away.
        """
        for point in path:
            dist = math.hypot(point[0] - x, point[1] - y)
            if dist >= self.Ld:
                return point
        # If reach the end of the path, aim for the final point
        return path[-1] if path else (x, y)

    def compute_velocity_commands(self, current_x, current_y, current_yaw, path):
        if not path:
            return 0.0, 0.0  # Stop if no path

        # 1. Find the target point
        target_x, target_y = self._get_lookahead_point(current_x, current_y, path)

        # 2. Calculate the relative angle (alpha) to the target
        dx = target_x - current_x
        dy = target_y - current_y
        target_angle = math.atan2(dy, dx)
        alpha = target_angle - current_yaw

        # Normalize alpha to be between -pi and pi
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi

        # 3. Calculate the required curvature to hit the target
        curvature = (2 * math.sin(alpha)) / self.Ld

        # 4.enforce turning radius constraint by capping curvature
        if abs(curvature) > self.max_curvature:
            curvature = math.copysign(self.max_curvature, curvature)

        # 5. Calculate final output commands
        linear_v = self.target_speed
        
        # Optional: Slow down dynamically during tighter turns to reduce slip on regolith
        linear_v *= (1.0 - (abs(curvature) / self.max_curvature) * 0.4) 

        # Angular velocity (omega) = v * k
        angular_v = linear_v * curvature

        return linear_v, angular_v