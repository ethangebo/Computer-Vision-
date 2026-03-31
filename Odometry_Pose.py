import math
import config as C

EFFECTIVE_TRACK_WIDTH = C.TRACK_WIDTH_M * C.SKID_STEER_FACTOR

class VelocityOdometry:
    #Output variables: center_odometry, yaw_odometry.
    def __init__(self, x=0.0, y=0.0, z=0.0, yaw_deg=0.0):
        # Initial local pose
        self.center_odometry = [x, y, z] 
        self.yaw_odometry = yaw_deg 
        self.last_time = None
    # Convert motor RPM to linear wheel velocity (meters per second)
    def rpm_to_velocity(self, rpm: float, side_inversion: int) -> float:

        # RPM -> RPS -> Wheel Velocity
        wheel_rps = (rpm / C.GEAR_RATIO) / 60.0
        velocity = 2.0 * math.pi * C.WHEEL_RADIUS_M * wheel_rps 
        return side_inversion * velocity 
    
    # Combine individual wheel velocities into left and right side velocities
    def aggregate_side_velocities(self, wheel_rpms: dict) -> tuple:
        left_wheels  = ["FL", "ML", "RL"] 
        right_wheels = ["FR", "MR", "RR"]

        left_data = []
        right_data = []

        for w in left_wheels:
            if w in wheel_rpms:
                v = self.rpm_to_velocity(wheel_rpms[w], C.LEFT_SIDE_INVERSION)
                left_data.append((v, C.WHEEL_WEIGHTS[w]))
        for w in right_wheels:
            if w in wheel_rpms:
                v = self.rpm_to_velocity(wheel_rpms[w], C.RIGHT_SIDE_INVERSION)
                right_data.append((v, C.WHEEL_WEIGHTS[w]))


        if not left_data or not right_data: 
            return 0.0, 0.0

        # Step 1: Reject extreme outliers
        def reject_extremes(data):
            if len(data) <= 2:
                return data

            # 1. Find the median velocity
            data_sorted = sorted(data, key=lambda x: x[0])
            median_v = data_sorted[len(data_sorted)//2][0]

            # 2. Keep wheels that are within a safe tolerance of the median
            TOLERANCE = 0.15
            filtered = [(v, w) for v, w in data if abs(v - median_v) <= TOLERANCE]

            # 3. Fallback: If all wheels are rejected, use the median wheel (which is likely correct)
            if not filtered:
                return [data_sorted[len(data_sorted)//2]]
            
            return filtered

        left_filtered = reject_extremes(left_data)
        right_filtered = reject_extremes(right_data)

        # STEP 2: weighted average
        def weighted_avg(data):
            total_v = sum(v * w for v, w in data)
            total_w = sum(w for _, w in data)
            return total_v / total_w if total_w > 0 else 0.0
        v_final_left = weighted_avg(left_filtered)
        v_final_right = weighted_avg(right_filtered)
        
        return v_final_left, v_final_right
    
    # Calculate odometry based on wheel velocities and elapsed time
    def update(self, wheel_rpms: dict, current_time: float):
 
        if self.last_time is None:
            self.last_time = current_time
            return self.center_odometry, self.yaw_odometry

        dt = current_time - self.last_time

        if dt < -40000: # e.g., dropping from 12:59 to 01:00
            dt += (12 * 3600)
            
        self.last_time = current_time
        
        if dt <= 0: return self.center_odometry, self.yaw_odometry

        # 1. Get velocities
        v_l, v_r = self.aggregate_side_velocities(wheel_rpms)

        # 2. Compute Body Motion using midpoint integration
        d_center = 0.5 * (v_l + v_r) * dt
        omega = (v_r - v_l) / EFFECTIVE_TRACK_WIDTH
        # Clamp to realistic rover capability (slip guard)
        omega = max(-C.MAX_YAW_RATE_RAD_S, min(C.MAX_YAW_RATE_RAD_S, omega))

        d_yaw_rad = omega * dt

        yaw_rad = math.radians(self.yaw_odometry)
        yaw_mid = yaw_rad + 0.5 * d_yaw_rad

        # 3. Calculate displacements
        dx = d_center * math.cos(yaw_mid)
        dy = d_center * math.sin(yaw_mid)
        d_yaw_deg = math.degrees(d_yaw_rad)

        # 4. Apply to local pose variables
        self.center_odometry[0] += dx
        self.center_odometry[1] += dy
        

        self.yaw_odometry = (self.yaw_odometry + d_yaw_deg)

        return self.center_odometry, self.yaw_odometry