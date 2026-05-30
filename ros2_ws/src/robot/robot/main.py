from __future__ import annotations
import time, sqlite3, os, math
from datetime import datetime

from robot.robot import Robot
from robot.hardware_map import Button
from robot.robot_fsm import RobotFSM


# More helpers
# TODO: change path when complete
from robot.fsm_helpers.firmware_helpers import configure_robot, start_robot, reset_mission_pose, home_lift, home_gripper
from robot.fsm_helpers.task_planner import tasks
from robot.fsm_helpers.task import build_task, Task

LOG_DIR = "/runtime_output/logs"

class MissionFSM(RobotFSM):
    def __init__(self, robot: Robot, task_list: list[dict]):
        super().__init__(robot, initial_state_str="INIT")

        # Tasks
        self.tasks = task_list
        self.current_task: Task = None
        self.task_idx = 0

        # Motion handles (mostly managed by tasks now, but kept for homing)
        self.homing_lift_handle = None
        self.homing_gripper_handle = None

        # Execution
        self.timer_start = None
        self.execution_print_period = 1.0
        
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
        if self.current_task:
            self.current_task.on_exit()
        
        self.task_idx += 1
        print(f"Advancing to task {self.task_idx}...")
        self._log(event="ADVANCE_TASK")

    def on_enter(self, state: str) -> None:
        self._log(event=f"ENTER_{state}")
        self.robot.was_button_pressed(button_id=Button.BTN_1, consume=True)
        self.robot.was_button_pressed(button_id=Button.BTN_2, consume=True)

        if state == "INIT":
            self.task_idx = 0
            self.current_task = None
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
                task_dict = self.tasks[self.task_idx]
                self.current_task = build_task(self.robot, task_dict)
                self.current_task.on_enter()
            else:
                self.trigger("to_done")

        elif state == "DONE":
            self.robot.shutdown()
            if self.conn:
                self.conn.close()
                self.conn = None
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


def run(robot: Robot) -> None:
    configure_robot(robot)
    fsm = MissionFSM(robot, tasks)
    fsm.on_enter("INIT")
    fsm.spin()
