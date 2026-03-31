import math
import heapq
import time
import threading
import config as C

class PlannerCommand:
    STOP = "STOP"
    EXCAVATE = "EXCAVATE"
    DEPOSIT = "DEPOSIT"


class RoverGlobalPlanner:

    def __init__(self, occupancy_grid):

        self.grid = occupancy_grid

        # Config Parameters
        self.res = C.GLOBAL_GRID_RES_M
        self.step_size = C.STEP_SIZE_M
        self.min_turn_radius = C.MIN_TURNING_RADIUS_M
        self.rover_length = C.ROVER_LENGTH_M
        self.rover_width = C.ROVER_WIDTH_M
        self.safety_margin = C.PLANNER_SAFETY_MARGIN_M
        self.max_planning_time = C.MAX_PLANNING_TIME_S

        self.goal_position_tolerance = C.GOAL_POS_TOLERANCE_M
        self.goal_yaw_tolerance = C.GOAL_YAW_TOLERANCE_RAD

        # Internal State
        self.current_pose = None
        self.current_command = PlannerCommand.STOP
        self.current_goal = None
        self.current_path = []

        # Permanent goal lists
        self.excavation_goals = [
            (2.0, 1.0, 0.0),
            (2.0, 1.5, 0.0),
        ]
        
        self.deposition_goals = [
            (-2.0, -1.0, math.pi),
            (-2.0, -1.5, math.pi),
        ]

        # Boundary limits
        margin = (self.rover_width / 2) + self.safety_margin
        self.safe_w = (self.grid.width_m / 2) - margin
        self.safe_h = (self.grid.height_m / 2) - margin

        # Calculate log-odds threshold for O(1) collision checks (prob = 0.6)
        self.occ_prob_threshold = C.GLOBAL_OCC_PROB_THRESHOLD
        self.occ_logodds_thresh = math.log(self.occ_prob_threshold / (1.0 - self.occ_prob_threshold))
        
        # Threading State
        self.lock = threading.Lock()
        self.running = False
        self.planner_thread = None

    def start(self):
        self.running = True
        self.planner_thread = threading.Thread(target=self._planning_loop, daemon=True)
        self.planner_thread.start()

    def stop(self):
        self.running = False
        if self.planner_thread:
            self.planner_thread.join()
            
    def update_pose(self, pose):
        with self.lock:
            self.current_pose = pose

    def set_command(self, command):

        with self.lock:
            if command != self.current_command:
                self.current_command = command
                self.current_goal = None
                self.current_path = []

    def get_current_path(self):
        with self.lock:
            return list(self.current_path)

    def get_status(self):

        if self.current_command == PlannerCommand.STOP:
            return "STOPPED"

        if self.current_goal is None:
            return "NO_GOAL_AVAILABLE"

        return "ACTIVE"

    # Main Update Loop
    def _planning_loop(self):
        while self.running:
            # 1. Safely grab state and manage goals inside the lock
            with self.lock:
                if self.current_pose is None or self.current_command == PlannerCommand.STOP:
                    self.current_goal = None
                    self.current_path = []
                    time.sleep(0.05)
                    continue

                if self.current_goal and self._goal_reached(self.current_pose, self.current_goal):
                    self._remove_current_goal()
                    self.current_goal = None
                    self.current_path = []
                    continue

                if self.current_goal is None:
                    self._select_next_goal()

                if self.current_goal is None:
                    self.current_path = []
                    time.sleep(0.05)
                    continue

                if not self._is_goal_valid(self.current_goal):
                    self._remove_current_goal()
                    self.current_goal = None
                    self.current_path = []
                    continue

                # Snapshot the variables needed for the search
                pose_snapshot = self.current_pose
                goal_snapshot = self.current_goal
                command_snapshot = self.current_command

            # 2. run planner outside of lock
            path = self._run_planner(pose_snapshot, goal_snapshot)

            # 3. Safely apply the result inside the lock
            with self.lock:
                # Only apply path if the command hasn't changed while calculating
                if self.current_command == command_snapshot:
                    self.current_path = path if path else []

    # Goal Management
    def _select_next_goal(self):

        if self.current_command == PlannerCommand.EXCAVATE:
            queue = self.excavation_goals
        elif self.current_command == PlannerCommand.DEPOSIT:
            queue = self.deposition_goals
        else:
            return

        while queue:
            candidate = queue[0]

            if self._is_goal_valid(candidate):
                self.current_goal = candidate
                return
            else:
                # Permanently remove invalid goal
                queue.pop(0)

        self.current_goal = None

    def _remove_current_goal(self):

        if self.current_command == PlannerCommand.EXCAVATE:
            queue = self.excavation_goals
        elif self.current_command == PlannerCommand.DEPOSIT:
            queue = self.deposition_goals
        else:
            return

        if queue and queue[0] == self.current_goal:
            queue.pop(0)

    # Goal Checking
    def _goal_reached(self, pose, goal_pose):

        dx = pose[0] - goal_pose[0]
        dy = pose[1] - goal_pose[1]

        if dx*dx + dy*dy > self.goal_position_tolerance ** 2:
            return False

        yaw_error = abs(math.atan2(
            math.sin(pose[2] - goal_pose[2]),
            math.cos(pose[2] - goal_pose[2])
        ))

        return yaw_error < self.goal_yaw_tolerance

    def _is_goal_valid(self, goal_pose):

        x, y, _ = goal_pose
        r, c = self.grid.world_to_grid(x, y)

        if not self.grid.in_bounds(r, c):
            return False

        return self.grid.L[r, c] < self.occ_logodds_thresh

    # Path Planner
    def _run_planner(self, start_pose, goal_pose):

        if self._goal_reached(start_pose, goal_pose):
            return [start_pose]

        start_time = time.time()
        queue = [(0, start_pose, [start_pose], 1)]
        visited = {}

        while queue:

            if time.time() - start_time > self.max_planning_time:
                return []

            priority, curr, path, last_dir = heapq.heappop(queue)

            if self._goal_reached(curr, goal_pose):
                return path

            state = (
                round(curr[0], 1),
                round(curr[1], 1),
                round(curr[2] / (math.pi / 18))
            )

            cost = len(path) * self.step_size

            if state in visited and visited[state] <= cost:
                continue

            visited[state] = cost

            for direction in [1, -1]:

                reverse_penalty = 2.5
                switch_penalty = 2.0 if direction != last_dir else 0.0
                move_cost = self.step_size if direction == 1 else self.step_size * reverse_penalty

                for steer in [0, 1, -1]:

                    next_pose = self._get_next_pose(curr, steer, direction)

                    if not self._is_collision(next_pose):

                        dx = next_pose[0] - goal_pose[0]
                        dy = next_pose[1] - goal_pose[1]
                        h = math.sqrt(dx*dx + dy*dy)

                        new_cost = cost + move_cost + switch_penalty
                        new_priority = new_cost + 2.0 * h

                        heapq.heappush(queue, (
                            new_priority,
                            next_pose,
                            path + [next_pose],
                            direction
                        ))

        return []

    # Motion Model
    def _get_next_pose(self, pose, steer, direction):

        x, y, yaw = pose
        d = self.step_size * direction

        if steer == 0:
            return (
                x + d * math.cos(yaw),
                y + d * math.sin(yaw),
                yaw
            )

        turn_angle = (d / self.min_turn_radius) * steer
        new_yaw = yaw + turn_angle
        avg_yaw = yaw + turn_angle / 2

        return (
            x + d * math.cos(avg_yaw),
            y + d * math.sin(avg_yaw),
            new_yaw
        )

    # Collision Checking
    def _is_collision(self, pose):

        x, y, _ = pose

        if abs(x) > self.safe_w or abs(y) > self.safe_h:
            return True

        r, c = self.grid.world_to_grid(x, y)

        if not self.grid.in_bounds(r, c):
            return True

        return self.grid.L[r, c] > self.occ_logodds_thresh