# config.py
# D455 → Mini PC → perception → plan → myRIO (motors). UWB localization.

# ===============================
# Camera / Depth 
# ===============================
FPS = 30
DEPTH_RES = (848, 480)

# Depth range to consider (meters)
Z_MIN_M = 0.15
Z_MAX_M = 4.0

# Rear camera (for reversing). Set to serial string when second D455 is connected, else None.
REAR_CAMERA_SERIAL = None
DEBUG_FORCE_REAR_CAMERA = False # rear camera toggle for debugging

# ===============================
# ROI (optional)
# ===============================
USE_ROI = True
ROI_TOP  = 0.30
ROI_SIDE = 0.15

# ===============================
# Mapping / Point sampling
# ===============================
# Sample every N pixels in each direction (4 => ~1/16 of pixels)
POINT_STRIDE = 4

# Obstacle filtering (reduce noise, redundant rays)
OBSTACLE_MIN_CLUSTER_SIZE = 2   # min points per grid cell to keep (drops isolated noise)
OBSTACLE_DOWNSAMPLE_RES_M = 0.05  # bucket obstacles into this resolution; one rep per cell

# Simple ground removal thresholds (RealSense deprojection convention)
# X = lateral, Y = vertical, Z = forward distance
GROUND_Y_MAX_M    = 0.10   # treat Y <= this as ground-ish (not used in the simple obstacle mask below, but kept)
OBSTACLE_Y_MIN_M  = 0.12   # keep points with Y > this as obstacle candidates (may need sign flip depending on mount)

# ===============================
# Occupancy Grid (local costmap)
# ===============================
GRID_RES_M = 0.10
GRID_WIDTH_M = 8.0
GRID_HEIGHT_M = 8.0

# Log-odds update tuning
P_OCC  = 0.70
P_FREE = 0.30
L_MIN  = -3.0
L_MAX  =  3.0

# Inflate obstacles for rover footprint
ROVER_RADIUS_M = 0.30
INFLATE_OCC_THRESHOLD = 0.65

# ===============================
# UI / Visualization
# ===============================
WINDOW_NAME = "Rover: Occupancy Grid + Depth"
SHOW_DEPTH_VIEW = True
SHOW_GRID_VIEW  = True
# Draw boxes around boulders (green) and craters (cyan) on depth view for testing
SHOW_BOXES = True
# Set True for headless on Beelink at competition (no OpenCV windows)
RUN_HEADLESS = False

PLANE_FIT_ROWS_FRAC = 0.40
PLANE_SAMPLE_MAX = 8000
PLANE_RANSAC_ITERS = 200
PLANE_INLIER_TOL_M = 0.03
PLANE_REFIT_EVERY = 3

#Objects 
GROUND_BAND_M = 0.02 #threshold for uneven ground surface
MIN_OBSTACLE_HEIGHT_M = 0.06
MAX_OBSTACLE_HEIGHT_M = 1.50

# Craters (negative obstacles): below-ground depressions
CRATER_HEIGHT_BELOW_M = 0.05   # points with h < -this are crater
MAX_CRATER_DEPTH_M = 0.35      # ignore h < -this (noise / far floor)

# Wall filter (exclude wall-like obstacles from grid)
FILTER_WALLS = True            # drop wall-like clusters (merged footprint or dense points)
WALL_FOOTPRINT_MAX_M = 0.70   # merged group larger than this (x or y extent) = wall, excluded
WALL_MERGE_DIST_M = 0.12      # merge clusters within this distance; then check merged footprint
WALL_MAX_POINTS_PER_CLUSTER = 50   # single cluster with more points than this = dense wall, excluded

# ===============================
# Comms (Mini PC → myRIO)
# ===============================
COMMS_MOCK = True   # False when myRIO wired
COMMS_RATE_HZ = 10
# COMMS_MYRIO_IP = "192.168.1.2"
# COMMS_MYRIO_PORT = 5555

# ===============================
# Global map (arena frame, UWB pose)
# ===============================
GLOBAL_GRID_WIDTH_M = 6.88
GLOBAL_GRID_HEIGHT_M = 5.0
GLOBAL_GRID_RES_M = 0.10   # coarser than local
GLOBAL_DECAY_AMOUNT = 0.0  # persistent; no decay
SHOW_GLOBAL_GRID = False   # toggle with 'm'
DEBUG_UWB_POSE_EVERY_N = 30  # set to 30 or 60 to print UWB pose + stamped count for debugging

# KSC arena static obstacle: column in Obstacle Zone (hard-coded so planner avoids it)
# Coordinates in same frame as global grid (origin at grid center: x ±3.44, y ±2.5 for 6.88×5 m)
# Adjust from rulebook/diagram; column ~ boulder width, so radius ~0.25 m is safe.
ENABLE_ARENA_COLUMN = True
ARENA_COLUMN_CENTER_X_M = 1.2   # approximate; tune to match UWB/arena frame
ARENA_COLUMN_CENTER_Y_M = 1.0
ARENA_COLUMN_RADIUS_M = 0.30    # ~boulder width; add margin for safety

# ===============================
# Kinematics & Odometry
# ===============================
WHEEL_RADIUS_M = 0.16256
TRACK_WIDTH_M = 0.623189
SKID_STEER_FACTOR = 1.35
GEAR_RATIO = 45.0
LEFT_SIDE_INVERSION = 1
RIGHT_SIDE_INVERSION = 1
MAX_YAW_RATE_RAD_S = 0.6 
WHEEL_WEIGHTS = {"FL": 0.7, "ML": 1.0, "RL": 0.7, "FR": 0.7, "MR": 1.0, "RR": 0.7}

# Wheelbase Offsets (Distance from Geometric Center)
OFFSET_FRONT_M = 0.31995618
OFFSET_MID_M   = 0.04849368
OFFSET_REAR_M  = 0.40810688

# Complementary Fusion Tuning
ALPHA_POS = 0.05
ALPHA_YAW = 0.08

# ===============================
# Planners (Global, Local, Reactive) & Control Limits
# ===============================
# 1. Rover Dimensions
ROVER_LENGTH_M = 1.22
ROVER_WIDTH_M = 0.71

# 2. Speeds & Kinematics
TARGET_SPEED_M_S = 0.6
MIN_TURNING_RADIUS_M = 1.5

# 3. Global Path Planner Tuning
STEP_SIZE_M = 0.15
PLANNER_SAFETY_MARGIN_M = 0.25
MAX_PLANNING_TIME_S = 0.15
GOAL_POS_TOLERANCE_M = 0.25
GOAL_YAW_TOLERANCE_RAD = 0.4
GLOBAL_OCC_PROB_THRESHOLD = 0.6

# 4. Local Path Planner Tuning
LOCAL_LOOKAHEAD_M = 0.8

# 5. Reactive Planner Tuning
REACTIVE_LOOKAHEAD_M = 1.0

# ===============================
# Dynamic Velocity Shifts
# ===============================
# Calculates maximum limits based on the planner settings above.
MAX_LINEAR_VEL = TARGET_SPEED_M_S
MAX_ANGULAR_VEL = TARGET_SPEED_M_S / MIN_TURNING_RADIUS_M

VEL_SHIFT_LINEAR = MAX_LINEAR_VEL
VEL_SHIFT_ANGULAR = MAX_ANGULAR_VEL

# ===============================
# TCP / myRIO Communication
# ===============================
MYRIO_IP = "192.168.49.11"
MYRIO_PORT = 65432

# ===============================
# UWB / Beacon Localization
# ===============================
UWB_PORTS = {
    "Beacon_1": "COM12", 
    "Beacon_2": "COM13"
}
UWB_BAUD_RATE = 115200

# Tag geometry on the rover (Meters)
UWB_DX_TAGS_M = 0.61 
UWB_DY_TAGS_M = 0.64
UWB_DZ_TAGS_M = 0.33

# Sync and timeouts
UWB_MAX_SKEW_S = 0.25
UWB_MAX_AGE_S = 0.5

# Anchor positions in the Arena (X, Y, Z in meters)
UWB_ANCHOR_POS = {
    "A": (2.0, 0.0, 0.42), 
    "B": (1.0, 0.0, 1.02),
    "C": (0.0, 1.0, 0.42), 
    "D": (0.0, 2.0, 1.02),
}

# MAC Address to Anchor mapping
UWB_ANCHOR_MAP = {
    "0A91": "A", 
    "D593": "B", 
    "16A7": "C", 
    "CF33": "D"}