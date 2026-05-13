from __future__ import annotations
import time

from robot.hardware_map import Button
from robot.robot import Robot
from robot.robot_fsm import RobotFSM

# More helpers
# TODO: change path when complete
from robot.fsm_helpers.vision_helpers import find_traffic_light_color
from robot.fsm_helpers.firmware_helpers import configure_robot, start_robot, home_lift
"""Uncomment this when performing full FSM, otherwise just play with tasks below"""
# from robot.fsm_helpers.task_planner import tasks
tasks = [
    {"state": "WAIT", "trigger": "green_light"},
    {"state": "NAV",  "visualTarget": "burger_bun", "goal_pose_mm": (5*610, 0, 90)},
]

# TODO: Move to some appropriate place
ENABLE_CAM = False
TOLERANCE_MM = 70
VELOCITY = 200.0


class MissionFSM(RobotFSM):
    def __init__(self, robot: Robot, task_list: list[dict]):
        super().__init__(robot, initial_state="INIT")
        self.tasks = task_list
        self.task_idx = 0
        self.drive_handle = None
        self.timer_start = None
        
        # Condensed transitions
        self.add_transition("INIT", "start", "EXECUTE")
        self.add_transition("EXECUTE", "next", "EXECUTE", action=self._advance_task)
        self.add_transition("EXECUTE", "done", "DONE")
        self.add_transition("EXECUTE", "error", "INIT")

    def _advance_task(self) -> None:
        self.task_idx += 1
        print(f"Advancing to task {self.task_idx}...")

    def on_enter(self, state: str) -> None:
        if state == "EXECUTE":
            self.timer_start = None # Reset timer for the new task
            self.drive_handle = None # Reset drive handle
            if self.task_idx < len(self.tasks):
                print(f"\n>>> EXECUTING TASK {self.task_idx}: {self.tasks[self.task_idx]}")
            else:
                self.trigger("done")

    def update(self) -> None:
        """Called at DEFAULT_FSM_HZ, non-blocking."""
        state_str = self.get_state_str()
        
        if state_str == "INIT":
            if self.robot.was_button_pressed(Button.BTN_1):
                self.trigger("start")
        
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
            # Final state; could add logic to reset or wait for restart
            pass

    def _handle_wait(self, params: dict) -> None:
        if params.get("trigger") == "green_light":
            if not ENABLE_CAM and self.robot.was_button_pressed(Button.BTN_1):
                self.trigger("next")
            elif ENABLE_CAM and find_traffic_light_color(self.robot) == "green":
                self.trigger("next")

    def _handle_nav(self, params: dict) -> None:
        if "goal_pose_mm" in params:
            x, y, theta = params.get("goal_pose_mm")
            if self.drive_handle is None:
                print(f"Driving toward: {params.get('visualTarget')}...")
                # TODO: Switch to APF follower
                self.drive_handle = self.robot.move_to(
                    x=x,
                    y=y,
                    velocity=VELOCITY,
                    tolerance=TOLERANCE_MM,
                    blocking=False,
                    timeout=None
                )
        
            if self.drive_handle.is_done():
                # TODO: Add go to heading
                print(f"Goal reached: {params.get('goal_pose_mm')}")
                # TODO: Add camera subscription comparison to check
                self.trigger("next")
        else:
            # If no goal pose, just finish (e.g. pure visual alignment task)
            self.trigger("next")


def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    home_lift(robot) # TODO: Tune parameters
    # TODO: Home servo?
    # TODO: Validate sensors?
    
    fsm = MissionFSM(robot, tasks)
    fsm.spin() # Blocks until the program is terminated
