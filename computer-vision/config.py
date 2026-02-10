# config.py
# D455 → Mini PC → perception → plan → myRIO (motors). UWB localization.

# ===============================
# Camera / Depth 
# ===============================
FPS = 30
DEPTH_RES = (848, 480)

# Depth range to consider (meters)
Z_MIN_M = 0.15
Z_MAX_M = 6.0

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
# Planner (grid-based heading choice)
# ===============================
SECTOR_DEGS = (-35, -15, 0, 15, 35)
LOOKAHEAD_M = 2.0
CORRIDOR_HALF_WIDTH_M = 0.40
STOP_DIST_M = 0.60
SLOW_DIST_M = 1.00

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
# Localization (UWB beacons)
# ===============================
UWB_MOCK = True
# UWB_BAUD = 115200
# UWB_PORT = "COM3"

# ===============================
# Global map (arena frame, UWB pose)
# ===============================
GLOBAL_GRID_WIDTH_M = 20.0
GLOBAL_GRID_HEIGHT_M = 10.0
GLOBAL_GRID_RES_M = 0.10   # coarser than local
GLOBAL_DECAY_AMOUNT = 0.0  # persistent; no decay
SHOW_GLOBAL_GRID = False   # toggle with 'm'

#beacons 
UWB_MOCK = False
UWB_PORTS = {
    "Beacon_1": "COM3",
    "Beacon_2": "COM4",
}
UWB_BAUD = 115200