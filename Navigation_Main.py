import time
import math
import socket
import datetime
import config as C
from Odometry_Pose import VelocityOdometry
from Beacon_Pose import BeaconPose
from Complementary_Fusion import ComplementaryFusion

# Planners
from Global_Path_Planner import RoverGlobalPlanner, PlannerCommand
from Local_Path_Planner import ConstrainedPurePursuit
from Reactive_Planner import ReactivePlanner

# Grid
from global_grid import GlobalOccupancyGrid2D

from Vision_System import VisionSystem

# Placeholder Wheel RPMs (Replace with motor feedback later)
wheel_rpms = {
    "FL": 0.0, "ML": 0.0, "RL": 0.0,
    "FR": 0.0, "MR": 0.0, "RR": 0.0
}

# Initialize Localization
odom = VelocityOdometry()
beacon = BeaconPose()
fusion = ComplementaryFusion()

beacon.start()

# Initialize Grids
global_grid = GlobalOccupancyGrid2D(width_m=6.88, height_m=5.0, res_m=0.1)

# Hard-code KSC arena column so global planner never routes through it
if getattr(C, "ENABLE_ARENA_COLUMN", False):
    n_col = global_grid.mark_occupied_region(
        C.ARENA_COLUMN_CENTER_X_M, C.ARENA_COLUMN_CENTER_Y_M, C.ARENA_COLUMN_RADIUS_M
    )
    print(f"Arena column marked: {n_col} cells")

vision = VisionSystem(global_grid)
vision.start()

# Initialize Planners
global_planner = RoverGlobalPlanner(global_grid)
local_planner = ConstrainedPurePursuit()
reactive_planner = ReactivePlanner(vision.local_grid)

# Initialize TCP Connection to myRIO
MYRIO_IP = "192.168.49.11"
MYRIO_PORT = 65432
myrio_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    myrio_socket.connect((MYRIO_IP, MYRIO_PORT))
    myrio_socket.setblocking(False)
except Exception as e:
    myrio_socket = None

# Persistent State Variables
tcp_buffer = ""
current_command = PlannerCommand.STOP # Defaults to STOP until told otherwise
wheel_rpms = {"FL": 0.0, "ML": 0.0, "RL": 0.0, "FR": 0.0, "MR": 0.0, "RR": 0.0}
global_planner.start()

def parse_myrio_time(t_str):
    """Converts HHMMSSLLL into total seconds."""
    if not t_str or len(t_str) != 9: 
        return None
    hh = int(t_str[0:2])
    mm = int(t_str[2:4])
    ss = int(t_str[4:6])
    lll = int(t_str[6:9])
    
    # Total seconds = Hours + Minutes + Seconds + Milliseconds
    return (hh * 3600) + (mm * 60) + ss + (lll / 1000.0)

try:
    while True:
        current_time = time.time()

        # Read Incoming TCP Data (Non-blocking)
        if myrio_socket is not None:
                    try:
                        # Read up to 1024 bytes from the network buffer
                        incoming_data = myrio_socket.recv(1024).decode('utf-8')
                        tcp_buffer += incoming_data
                        
                        # Process all complete messages (separated by \n)
                        while '\n' in tcp_buffer:
                            complete_message, tcp_buffer = tcp_buffer.split('\n', 1)
                            
                            # Example message 1: "CMD:EXCAVATE"
                            # Example message 2: "FL:12.5,ML:12.5,RL:12.5,FR:12.5,MR:12.5,RR:12.5,T:041530550"
                            parts = complete_message.split(',')
                            for part in parts:
                                if ':' not in part: continue
                                key, value = part.split(':', 1)
                                
                                # Only update what actually arrived
                                if key == "CMD":
                                    if value == "STOP": current_command = PlannerCommand.STOP
                                    elif value == "EXCAVATE": current_command = PlannerCommand.EXCAVATE
                                    elif value == "DEPOSIT": current_command = PlannerCommand.DEPOSIT
                                elif key in wheel_rpms:
                                    wheel_rpms[key] = float(value)
                                elif key == "T":
                                    last_myrio_timestamp = value # <-- CAPTURE TIMESTAMP
                    except BlockingIOError:
                        pass # No new data this loop
                    except Exception as e:
                        print(f"TCP read error: {e}")


        # 1. Localization
        # Parse hardware time, fallback to PC time if no TCP data yet
        hardware_time = parse_myrio_time(last_myrio_timestamp)
        odom_time = hardware_time if hardware_time is not None else current_time

        odom_center, odom_yaw = odom.update(wheel_rpms, odom_time)
        beacon_center, beacon_yaw = beacon.get_update()
        fused_center, fused_yaw = fusion.update(
            odom_center, odom_yaw, beacon_center, beacon_yaw
        )
        fused_pose = (fused_center[0], fused_center[1], fused_yaw)

        # Sync Vision with latest Pose for Global Stamping
        vision.update_pose(fused_pose[0], fused_pose[1], math.radians(fused_pose[2]))
        
        # 2. Global Planning
        global_planner.update_pose(fused_pose)
        global_planner.set_command(current_command)
        
        current_path = global_planner.get_current_path()

        final_v = 0.0
        final_w = 0.0
        # 3. Local Planning
        current_x, current_y, current_yaw = fused_pose
        target_v, target_w = local_planner.compute_velocity_commands(
            current_x, current_y, current_yaw, current_path
        )
        # Switch to rear camera when reversing (so reactive planner sees obstacles behind)
        vision.update_reversing(target_v < 0)
        # 4. Reactive Planning
        if current_command == PlannerCommand.STOP:
            final_v, final_w = 0.0, 0.0
        elif reactive_planner.is_trajectory_blocked(target_v, target_w):
            print("EMERGENCY: Intended arc is blocked by rock/pit!")
            final_v, final_w = 0.0, 0.0
        else:
            final_v, final_w = target_v, target_w
        
        #Shifts velocities into positive v/w range for myRIO (which applies its own negative shift for rear motor control)
        shifted_v = final_v + C.VEL_SHIFT_LINEAR
        shifted_w = final_w + C.VEL_SHIFT_ANGULAR
        
        # Hardware Communication
        if myrio_socket is not None:
            # Generate timestamp: HHMMSSLLL (12-hour format)
            now = datetime.datetime.now()
            # %I = 12-hour hour, %M = minute, %S = second. 
            # Microseconds // 1000 = milliseconds (padded to 3 digits)
            time_str = now.strftime("%I%M%S") + f"{now.microsecond // 1000:03d}"
            
            # Format: "V:0.60,W:-0.15,T:043015123\n"
            command_str = f"V:{shifted_v:.2f},W:{shifted_w:.2f},T:{time_str}\n"
            try:
                myrio_socket.sendall(command_str.encode('utf-8'))
            except Exception as e:
                # TCP Connection lost
                myrio_socket.close()
                myrio_socket = None # Prevent spamming errors

        # Run at ~50Hz to match odometry/CV updates
        time.sleep(0.02)

except KeyboardInterrupt:
    print("Shutting down navigation system...")

finally:
    if myrio_socket is not None:
        myrio_socket.close()
    vision.stop_vision()
    global_planner.stop()
    beacon.stop()