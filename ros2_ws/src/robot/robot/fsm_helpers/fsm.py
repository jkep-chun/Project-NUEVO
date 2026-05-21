from __future__ import annotations
import time, sqlite3, os
from datetime import datetime

from robot.robot import Robot
from robot.hardware_map import Button, Stepper, StepMoveType
from robot.robot_fsm import RobotFSM

# More helpers
# TODO: change path when complete
from robot.fsm_helpers.vision_helpers import find_traffic_light_color
from robot.fsm_helpers.firmware_helpers import configure_robot, start_robot, reset_mission_pose, home_lift, home_gripper
from robot.fsm_helpers.task_planner import tasks
from robot.fsm_helpers.burgerbot_parameters import TOLERANCE_MM, VELOCITY_MM_S, NavStage, ManipStage, PICK_SEQUENCE, PLACE_SEQUENCE
from robot.fsm_helpers.course_parameters import STEP_LEVEL_POSITION

LOG_DIR = "/runtime_output/logs"

ENABLE_CAM = False
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
        self.nav_stage = None
        self.manip_sequence = []
        self.manip_stage = None
        self.manip_idx = 0
        
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
                
                elif self.task_type == "MANIP":
                    cmd = self.task.get("cmd")
                    if cmd == "pick":
                        self.manip_sequence = PICK_SEQUENCE
                    elif cmd == "place":
                        self.manip_sequence = PLACE_SEQUENCE
                    self.manip_idx = 0

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
            if self.task_type == "WAIT":
                self._handle_wait(self.task)
            elif self.task_type == "NAV":
                self._handle_nav(self.task)
            # elif self.task_type == "MANIP":
            #     self._handle_manip(self.task)
            # elif self.task_type == "IDENT":
            #     self._handle_ident(self.task)

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

    def _handle_nav(self, params: dict) -> None:
        if "goal_pose_mm" not in params:
            self.trigger("next")
            return
        
        x, y, theta = self.robot.get_pose()
        time_now = time.monotonic()
        if time_now - self.timer_start >= self.execution_print_period:
            status = self.nav_handle.is_done() if self.nav_handle else "STARTING"
            print(f"[Task {self.task_idx}] stage: {self.nav_stage.name}, done: {status}, pose: ({x:.2f},{y:.2f},{theta:.2f})")
            self._log(event="TELEMETRY")
            self.timer_start = time_now
        g_x, g_y, g_theta = params.get("goal_pose_mm")

        if self.nav_stage == NavStage.POSITION:
            if self.nav_handle is None:
                print(f"Driving toward: ({g_x:.2f},{g_y:.2f})")
                self.nav_handle = self.robot.lapf_to_goal(
                    x=g_x,
                    y=g_y,
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
        
            if self.nav_handle.is_done():
                print(f"[Task {self.task_idx}] stage: {self.nav_stage.name}, done: {True}, pose: ({x:.2f},{y:.2f},{theta:.2f})")
                self._log(event="TELEMETRY")

                self.nav_stage = NavStage.HEADING
                self.nav_handle = None

        elif self.nav_stage == NavStage.HEADING:
            if self.nav_handle is None:
                print(f"Heading toward: {g_theta}")
                self.nav_handle = self.robot.turn_to(
                    angle_deg=g_theta,
                    blocking=False,
                    tolerance_deg=2.0,
                    timeout=None
                )

            if self.nav_handle.is_done():
                self.trigger("next")

    # def _handle_manip(self, params: dict) -> None:
    #     self.manip_stage = self.manip_sequence[self.manip_idx]

    #     if self.manip_handle is not None:
    #         if self.manip_handle.is_done():
    #             self.manip_handle = None
    #             self.manip_idx += 1
    #             if not self.manip_idx < len(self.manip_sequence):
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
