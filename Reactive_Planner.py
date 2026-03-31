import math
import config as C

class ReactivePlanner:
    def __init__(self, local_occupancy_grid):
        self.local_grid = local_occupancy_grid
        self.check_width = C.ROVER_WIDTH_M + 0.2 
        self.lookahead_dist = C.REACTIVE_LOOKAHEAD_M
        self.logodds_thresh = math.log(C.GLOBAL_OCC_PROB_THRESHOLD / (1.0 - C.GLOBAL_OCC_PROB_THRESHOLD))

    def is_trajectory_blocked(self, v, w):
        """
        Projects a curved arc based on linear velocity (v) and angular velocity (w).
        Checks if that specific path is blocked in the local grid.
        """
        # No movement, no path being blocked
        if abs(v) < 0.01:
            return False

        # Parameters for the projection
        time_step = 0.1  # Check every 100ms along the path
        total_time = self.lookahead_dist / v
        
        t = 0.0
        while t <= total_time:
            # Kinematic Projection (Where will the rover be at time 't'?)
            # Since this is in the rover frame, start is at (0,0,0)
            if abs(w) < 1e-5: # Driving straight
                x = v * t
                y = 0.0
            else: # Driving an arc
                # Standard circular motion math
                r_turn = v / w
                x = r_turn * math.sin(w * t)
                y = r_turn * (1 - math.cos(w * t))

            # Check a small cross-section of the rover's width at this projected point
            for lateral_offset in [-self.check_width/2, 0, self.check_width/2]:
                # Offset the check point by the rover's width
                check_x = x - lateral_offset * math.sin(w * t)
                check_y = y + lateral_offset * math.cos(w * t)

                r, c = self.local_grid.world_to_grid(check_x, check_y)
                
                if 0 <= r < self.local_grid.L.shape[0] and 0 <= c < self.local_grid.L.shape[1]:
                    if self.local_grid.L[r, c] > self.logodds_thresh:
                        return True
            
            t += time_step

        return False