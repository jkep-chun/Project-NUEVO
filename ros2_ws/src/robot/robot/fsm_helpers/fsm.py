from __future__ import annotations
import time, sqlite3, os, math
from datetime import datetime

from robot.robot import Robot
from robot.hardware_map import Button, Stepper, StepMoveType
from robot.robot_fsm import RobotFSM
from robot.util import densify_polyline


# More helpers
# TODO: change path when complete
from robot.fsm_helpers.firmware_helpers import configure_robot, start_robot, reset_mission_pose, home_lift, home_gripper
from robot.fsm_helpers.task_planner import tasks
from robot.fsm_helpers.burgerbot_parameters import TOLERANCE_MM, VELOCITY_MM_S, LOOKAHEAD_MM, SPACING_MM, NavStage, ManipStage, PICK_SEQUENCE, PLACE_SEQUENCE
from robot.fsm_helpers.course_parameters import STEP_LEVEL_POSITION

LOG_DIR = "/runtime_output/logs"

ENABLE_GRIPPER = False
ENABLE_LIFT = False

class MissionFSM(RobotFSM):
    def __init__(self, robot: Robot, task_list: list[dict]):
        super().__init__(robot, initial_state_str="INIT")

        # Tasks
        self.tasks = task_list
        self.task = None
        self.task_idx = None
        self.task_type = None

        # Motion handles
        self.homing_lift_handle = None
        self.homing_gripper_handle = None
        self.nav_handle = None
        self.manip_handle = None

        # Execution
        self.timer_start = None
        self.execution_print_period = 1.0
        self.nav_stage_sequence = []
        self.nav_stage = None
        self.nav_stage_idx = 0
        self.nav_waypoints = []
        self.nav_waypoints_segment = []
        self.nav_waypoints_idx = None
        self.nav_waypoints_idx_last = None
        self.nav_init = False
        self.manip_sequence = []
        self.manip_stage = None
        self.manip_stage_idx = 0
        
        # Transitions
        self.add_transition("INIT", "to_execute", "EXECUTE")
        self.add_transition("INIT", "to_homing", "HOMING")
        self.add_transition("HOMING", "to_init", "INIT")
        self.add_transition("EXECUTE", "next", "EXECUTE", action=self._advance_task)
        self.add_transition("EXECUTE", "to_done", "DONE")
        self.add_transition("EXECUTE", "error", "INIT")
        self.add_transition("DONE", "to_init", "INIT")

        # SQLITE3 LOGGING SETUP
        self.conn = None
        self._setup_logging()

    def _setup_logging(self) -> None:
        """Initialize sqlite3 database for logging."""
        os.makedirs(LOG_DIR, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        db_path = os.path.join(LOG_DIR, f"fsm_log_{timestamp}.db")
        print(f"Logging mission data to: {db_path}")
        
        self.conn = sqlite3.connect(db_path)
        cursor = self.conn.cursor()
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS fsm_log (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL,
                state TEXT,
                event TEXT,
                task_idx INTEGER,
                x REAL,
                y REAL,
                theta REAL
            )
        ''')
        self.conn.commit()

    def _log(self, event: str = "UPDATE") -> None:
        """Log current state and telemetry to sqlite3."""
        if self.conn is None:
            return
        
        x, y, theta = self.robot.get_pose()
        cursor = self.conn.cursor()
        cursor.execute('''
            INSERT INTO fsm_log (timestamp, state, event, task_idx, x, y, theta)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        ''', (time.time(), self.get_state_str(), event, self.task_idx, x, y, theta))
        self.conn.commit()

    def _advance_task(self) -> None:
        self.task_idx += 1
        print(f"Advancing to task {self.task_idx}...")
        self._log(event="ADVANCE_TASK")

    def on_enter(self, state: str) -> None:
        self._log(event=f"ENTER_{state}")
        self.robot.was_button_pressed(button_id=Button.BTN_1, consume=True)
        self.robot.was_button_pressed(button_id=Button.BTN_2, consume=True)

        if state == "INIT":
            self.task_idx = 0
            print("\n>>> INIT - STARTING ROBOT")
            start_robot(self.robot)
            reset_mission_pose(self.robot)
            print(
                "\n>>> INIT - FIRMWARE RUNNING" \
                "\n    BTN_1 -> EXECUTE" \
                "\n    BTN_2 -> HOMING"
            )

        elif state == "HOMING":
            self.homing_lift_handle = home_lift(self.robot)
            self.homing_gripper_handle = home_gripper(self.robot)
            print("\n>>> HOMING - BEGIN SEQUENCE")

        elif state == "EXECUTE":
            self.timer_start = time.monotonic() # Reset timer for the new task

            if self.task_idx < len(self.tasks):
                print(f"\n>>> EXECUTE - TASK {self.task_idx}: {self.tasks[self.task_idx]}")
                self.task = self.tasks[self.task_idx]
                self.task_type = self.task.get("state")

                if self.task_type == "NAV":
                    self.nav_handle = None # Reset drive handle
                    self.nav_stage = NavStage.POSITION
                    waypoints = self.task.get("waypoints")
                    # Ensure waypoints is a list of tuples/lists
                    if waypoints and not isinstance(waypoints[0], (list, tuple)):
                        waypoints = [waypoints]
                    self.nav_waypoints = densify_polyline(waypoints, spacing=SPACING_MM)
                    self.nav_waypoints_idx = 0
                    self.nav_waypoints_idx_last = 0
                    self.nav_init = True
                
                elif self.task_type == "MANIP":
                    cmd = self.task.get("cmd")
                    if cmd == "pick":
                        self.manip_sequence = PICK_SEQUENCE
                    elif cmd == "place":
                        self.manip_sequence = PLACE_SEQUENCE
                    self.manip_stage_idx = 0

            else:
                self.trigger("to_done")

        elif state == "DONE":
            self.robot.shutdown()
            if self.conn:
                self.conn.close()
                self.conn = None
            # self.robot.step_disable()
            # self.robot.disable_servo()
            print("\n>>> DONE - TASKS COMPLETE; PRESS BTN_1 TO REINITIALIZE")


    def update(self) -> None:
        """Called at DEFAULT_FSM_HZ, non-blocking."""
        state_str = self.get_state_str()
        
        if state_str == "INIT":
            if self.robot.was_button_pressed(Button.BTN_1):
                self.trigger("to_execute")
            elif self.robot.was_button_pressed(Button.BTN_2):
                self.trigger("to_homing")

        elif state_str == "HOMING":
             # Separate from if statement logic to ensure each .is_done is called
            lift_done = self.homing_lift_handle.is_done()
            grip_done = self.homing_gripper_handle.is_done()
            if lift_done and grip_done:
                self.homing_lift_handle = None
                self.homing_gripper_handle = None
                self.trigger("to_init")
        
        elif state_str == "EXECUTE":
            if self.current_task:
            self.current_task.update()
            if self.current_task.is_done():
                self.trigger("next")

        elif state_str == "DONE":
            if self.robot.was_button_pressed(Button.BTN_1):
                self._setup_logging()
                self.trigger("to_init")


    def _handle_wait(self, params: dict) -> None:
        if params.get("trigger") == "green_light":
            if not ENABLE_CAM:
                if self.robot.was_button_pressed(Button.BTN_1):
                    print("trigger - BTN_1 (debug)")
                    self.trigger("next")
            elif find_traffic_light_color(self.robot) == "green":
                print("trigger - green light detected")
                self.trigger("next")

    def _finish_segment(self) -> None:
        """Advance to the next segment or task after all stages of the current segment are complete."""
        self.nav_waypoints_idx = self.nav_waypoints_idx_last + 1
        
        if self.nav_waypoints_idx < len(self.nav_waypoints):
            self.nav_init = True
        else:
            print(f">>> Task {self.task_idx} COMPLETE")
            self.trigger("next")

    def _handle_nav(self, params: dict) -> None:
        if not params.get("waypoints"):
            print(f"[Task {self.task_idx}] WARNING: No waypoints provided, skipping.")
            self.trigger("next")
            return
        
        path_planner = params.get("path_planner")
        
        if self.nav_init:
            # Identify a "segment": waypoints up to and including the next heading target
            segment_start_idx = self.nav_waypoints_idx
            self.nav_waypoints_segment.clear()
            g_theta = []
            
            while self.nav_waypoints_idx < len(self.nav_waypoints):
                wp = self.nav_waypoints[self.nav_waypoints_idx]
                g_x, g_y = wp[0], wp[1]
                g_theta = wp[2:]
                self.nav_waypoints_segment.append((g_x, g_y))
                
                # If we found a theta, this is the end of the segment
                if g_theta:
                    break
                
                # Otherwise, keep going until the end of the list
                if self.nav_waypoints_idx < len(self.nav_waypoints) - 1:
                    self.nav_waypoints_idx += 1
                else:
                    # Reached the end of the waypoint list without finding a theta
                    break

            self.nav_waypoints_idx_last = self.nav_waypoints_idx
            print(f">>> [Task {self.task_idx}] New segment: indices {segment_start_idx} to {self.nav_waypoints_idx_last}")
            
            if g_theta:
                self.nav_stage_sequence = [NavStage.POSITION, NavStage.HEADING]
                self.nav_target_theta = float(g_theta[0])
            else:
                self.nav_stage_sequence = [NavStage.POSITION]
                self.nav_target_theta = None

            if path_planner == "pp":
                # Calculate heading to first waypoint of segment
                curr_x, curr_y, _ = self.robot.get_pose()
                first_wp = self.nav_waypoints_segment[0]
                dx = first_wp[0] - curr_x
                dy = first_wp[1] - curr_y
                if math.hypot(dx, dy) > 10.0: # Only turn if far enough away
                    self.nav_start_theta = math.degrees(math.atan2(dy, dx))
                    self.nav_stage_sequence.insert(0, NavStage.START_HEADING)

            if path_planner == "lapf":
                self.nav_waypoints_idx = segment_start_idx
            
            self.nav_stage_idx = 0
            self.nav_stage = self.nav_stage_sequence[self.nav_stage_idx]
            self.nav_init = False
            self.nav_handle = None
        
        x, y, theta = self.robot.get_pose()
        time_now = time.monotonic()
        if time_now - self.timer_start >= self.execution_print_period:
            status = self.nav_handle.is_done() if self.nav_handle else "IDLE"
            print(f"[Task {self.task_idx}] stage: {self.nav_stage.name}, wp_idx: {self.nav_waypoints_idx}, pose: ({x:.2f},{y:.2f},{theta:.2f})")
            self._log(event="TELEMETRY")
            self.timer_start = time_now

        if self.nav_stage == NavStage.START_HEADING:
            if self.nav_handle is None:
                print(f"Turning to face first waypoint: {self.nav_start_theta:.2f} deg")
                self.nav_handle = self.robot.turn_to(
                    angle_deg=self.nav_start_theta,
                    blocking=False,
                    tolerance_deg=5.0,
                    timeout=None
                )
                return

            if self.nav_handle.is_done():
                self.nav_handle = None
                self.nav_stage_idx += 1
                if self.nav_stage_idx < len(self.nav_stage_sequence):
                    self.nav_stage = self.nav_stage_sequence[self.nav_stage_idx]
                else:
                    self._finish_segment()

        elif self.nav_stage == NavStage.POSITION:
            if self.nav_handle is None:
                if path_planner == "pp":
                    print(f"Driving segment with {len(self.nav_waypoints_segment)} waypoints")
                    self.nav_handle = self.robot.purepursuit_follow_path(
                        waypoints=self.nav_waypoints_segment,
                        velocity=VELOCITY_MM_S,
                        lookahead=LOOKAHEAD_MM,
                        tolerance=TOLERANCE_MM,
                        blocking=False,
                        max_angular_rad_s=1.0,
                        advance_radius=LOOKAHEAD_MM
                    )
                elif path_planner == "lapf":
                    wp = self.nav_waypoints[self.nav_waypoints_idx]
                    tx, ty = wp[0], wp[1]
                    print(f"Driving (LAPF) toward: ({tx:.2f},{ty:.2f})")
                    self.nav_handle = self.robot.lapf_to_goal(
                        x=tx, y=ty,
                        velocity=VELOCITY_MM_S,
                        tolerance=TOLERANCE_MM,
                        leash_length_mm=50,
                        repulsion_range_mm=350.0,
                        target_speed_mm_s=200.0,
                        repulsion_gain=550.0,
                        attraction_gain=1.0,
                        force_ema_alpha=0.35,
                        inflation_margin_mm=150.0,
                        leash_half_angle_deg=25.0,
                        blocking=False
                    )
                return

            if self.nav_handle.is_done():
                self.nav_handle = None
                if path_planner == "lapf" and self.nav_waypoints_idx < self.nav_waypoints_idx_last:
                    self.nav_waypoints_idx += 1
                    return
                else:
                    self.nav_stage_idx += 1
                    if self.nav_stage_idx < len(self.nav_stage_sequence):    
                        self.nav_stage = self.nav_stage_sequence[self.nav_stage_idx]
                    else:
                        self._finish_segment()

        elif self.nav_stage == NavStage.HEADING:
            if self.nav_handle is None:
                print(f"Turning to: {self.nav_target_theta:.2f} deg")
                self.nav_handle = self.robot.turn_to(
                    angle_deg=self.nav_target_theta,
                    blocking=False,
                    tolerance_deg=2.0,
                    timeout=None
                )
                return

            if self.nav_handle.is_done():
                self.nav_handle = None
                self.nav_stage_idx += 1
                if self.nav_stage_idx < len(self.nav_stage_sequence):
                    self.nav_stage = self.nav_stage_sequence[self.nav_stage_idx]
                else:
                    self._finish_segment()

    # def _handle_manip(self, params: dict) -> None:
    #     self.manip_stage = self.manip_sequence[self.manip_stage_idx]

    #     if self.manip_handle is not None:
    #         if self.manip_handle.is_done():
    #             self.manip_handle = None
    #             self.manip_stage_idx += 1
    #             if not self.manip_stage_idx < len(self.manip_sequence):
    #                 self.trigger("next")

    #     if self.manip_stage == ManipStage.LEVEL:
    #         if self.manip_handle == None:
    #             self.manip_handle = self.robot.step_move(
    #                 stepper_id=Stepper.STEPPER_1, # TODO: Define elsewhere
    #                 step=STEP_LEVEL_POSITION,
    #                 move_type=StepMoveType.ABSOLUTE,
    #                 blocking=False,
    #                 timeout=None
    #             )

    #     elif self.manip_stage == ManipStage.OPEN:
    #         if self.manip_handle == None:
    #             self.manip_handle = self.robot.

    #     elif self.manip_stage == ManipStage.CLOSE:
    #     elif self.manip_stage == ManipStage.FORWARD:
    #     elif self.manip_stage == ManipStage.RETREAT:
    #     elif self.manip_stage == ManipStage.RAISE:
    #     elif self.manip_stage == ManipStage.LOWER:

def run(robot: Robot) -> None:
    configure_robot(robot)
    fsm = MissionFSM(robot, tasks)
    fsm.on_enter("INIT")
    fsm.spin()
