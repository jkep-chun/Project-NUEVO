from __future__ import annotations
import time

from robot.hardware_map import Button
from robot.robot import Robot
from robot.robot_fsm import RobotFSM

# More helpers
# TODO: change path when complete
from robot.fsm_helpers.vision_helpers import find_traffic_light_color
from robot.fsm_helpers.firmware_helpers import configure_robot, start_robot, home_lift
from robot.fsm_helpers.task_planner import tasks, TOLERANCE_MM, VELOCITY_MM_S

ENABLE_CAM = False
ENABLE_GRIPPER = False
ENABLE_LIFT = False

from enum import IntEnum, auto

class NavStage(IntEnum):
    POSITION = auto()
    HEADING = auto()

class MissionFSM(RobotFSM):
    def __init__(self, robot: Robot, task_list: list[dict]):
        super().__init__(robot, initial_state_str="INIT")
        self.tasks = task_list
        self.task_idx = None
        self.drive_handle = None
        self.homing_handle = None
        self.nav_stage = None
        # self.manip_stage = None
        self.timer_start = None
        self.execution_print_period = 1.0
        
        self.add_transition("INIT", "to_execute", "EXECUTE")
        self.add_transition("INIT", "to_homing", "HOMING")
        self.add_transition("HOMING", "to_init", "INIT")
        self.add_transition("EXECUTE", "next", "EXECUTE", action=self._advance_task)
        self.add_transition("EXECUTE", "to_done", "DONE")
        self.add_transition("EXECUTE", "error", "INIT")
        self.add_transition("DONE", "to_init", "INIT")

    def _advance_task(self) -> None:
        self.task_idx += 1
        print(f"Advancing to task {self.task_idx}...")

    def on_enter(self, state: str) -> None:
        if state == "INIT":
            self.task_idx = 0
            print("\n>>> INIT - STARTING ROBOT")
            start_robot(self.robot)
            print(
                "\n>>> INIT - FIRMWARE RUNNING" \
                "\n    BTN_1 -> EXECUTE" \
                "\n    BTN_2 -> HOMING"
            )

        elif state == "HOMING":
            self.homing_handle = None
            print("\n>>> HOMING - BEGIN SEQUENCE")

        elif state == "EXECUTE":
            self.timer_start = time.monotonic() # Reset timer for the new task
            self.drive_handle = None # Reset drive handle
            self.nav_stage = NavStage.POSITION
            # self.manip_stage = None
            if self.task_idx < len(self.tasks):
                print(f"\n>>> EXECUTE - TASK {self.task_idx}: {self.tasks[self.task_idx]}")
            else:
                self.trigger("to_done")

        elif state == "DONE":
            self.robot.shutdown()
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
            if self.homing_handle is None:
                self.homing_handle = home_lift(self.robot)
            
            if self.homing_handle.is_done():
                self.trigger("to_init")
        
        elif state_str == "EXECUTE":
            task = self.tasks[self.task_idx]
            task_type = task.get("state")

            if task_type == "WAIT":
                self._handle_wait(task)
            elif task_type == "NAV":
                self._handle_nav(task)
            # elif task_type == "MANIP":
            #     self._handle_manip(task)
            # elif task_type == "IDENT":
            #     self._handle_ident(task)

        elif state_str == "DONE":
            if self.robot.was_button_pressed(Button.BTN_1):
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
        
        time_now = time.monotonic()
        if time_now - self.timer_start >= self.execution_print_period:
            status = self.drive_handle.is_done() if self.drive_handle else "STARTING"
            print(f"[Task {self.task_idx}] stage: {self.nav_stage.name}, done: {status}")
            self.timer_start = time_now
        x, y, theta = params.get("goal_pose_mm")

        if self.nav_stage == NavStage.POSITION:
            if self.drive_handle is None:
                print(f"Driving toward: ({x/1000.0},{y/1000.0})")
                # TODO: Switch to APF or pure pursuit follower
                self.drive_handle = self.robot.move_to(
                    x=x,
                    y=y,
                    velocity=VELOCITY_MM_S,
                    tolerance=TOLERANCE_MM,
                    blocking=False,
                    timeout=None
                )
        
            if self.drive_handle.is_done():
                self.nav_stage = NavStage.HEADING
                self.drive_handle = None

        if self.nav_stage == NavStage.HEADING:
            if self.drive_handle is None:
                print(f"Heading toward: {theta}")
                self.drive_handle = self.robot.turn_to(
                    angle_deg=theta,
                    blocking=False,
                    tolerance_deg=2.0,
                    timeout=None
                )

            if self.drive_handle.is_done():
                self.trigger("next")


def run(robot: Robot) -> None:
    configure_robot(robot)
    fsm = MissionFSM(robot, tasks)
    fsm.on_enter("INIT")
    fsm.spin()
