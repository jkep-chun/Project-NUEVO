from __future__ import annotations
import time

from robot.hardware_map import Button
from robot.robot import FirmwareState, Robot
from robot.robot_fsm import RobotFSM

# More helpers
# TODO: change path when complete
from robot.fsm_helpers.firmware_helpers import configure_robot, start_robot
from robot.fsm_helpers.task_planner import tasks

# ---------------------------------------------------------------------------
# Robot build configuration
# ---------------------------------------------------------------------------

VELOCITY_MM_S = 80 # TODO: Tune
TIMEOUT_CAM_DETECTION = 3 # TODO: Tune
DEBUG_MODE = True


class MissionFSM(RobotFSM):
    def __init__(self, robot: Robot, task_list: list[dict]):
        super().__init__(robot, initial_state="INIT")
        self.tasks = task_list
        self.task_idx = 0
        self.drive_handle = None
        self.timer_start = None
        
        # Build transitions based on the task sequence.
        # We use unique state names (e.g., "NAV_1") to handle repeating task types.
        self.add_transition("INIT", "start", self._get_task_state_name(0), action=self._on_mission_begin)
        
        for i in range(len(self.tasks) - 1):
            self.add_transition(
                self._get_task_state_name(i), 
                "next", 
                self._get_task_state_name(i + 1)
            )

    def _get_task_state_name(self, index: int) -> str:
        return f"{self.tasks[index]['state']}_{index}"

    def _on_mission_begin(self):
        print("Mission sequence started!")

    def on_enter(self, state: str) -> None:
        """Hook called by RobotFSM whenever we transition to a new state."""
        if "_" in state:
            # Sync our internal index with the new state
            self.task_idx = int(state.split("_")[-1])
            self.timer_start = None # Reset timer for the new task
            self.drive_handle = None # Reset drive handle
            print(f"\n>>> ENTERING TASK {self.task_idx}: {self.tasks[self.task_idx]}")

    def update(self) -> None:
        """
        Core logic loop. Called at DEFAULT_FSM_HZ (e.g., 50Hz).
        This method must be non-blocking.
        """
        state_str = self.get_state()

        if state_str == "INIT":
            if self.robot.was_button_pressed(Button.BTN_1):
                self.trigger("start")
            return

        # Get current task definition
        task = self.tasks[self.task_idx]
        task_type = task["state"]

        if task_type == "WAIT":
            if task.get("trigger") == "green_light":
                if DEBUG_MODE:
                    # Simulated green light via Button 1
                    if self.robot.was_button_pressed(Button.BTN_1):
                        self.trigger("next")
                # TODO: Replace above w/ reading topic from Camera
                # if camera publishes w/ timeout:
                #     if msg == "green_light":
                #         self.trigger("next")
                #     elif msg == "red_light":
                #         pass
                # else:
                #     self.trigger("error")

        elif task_type == "NAV":
            if "goal_pose_mm" in task:
                x, y, theta = task.get("goal_pose_mm")
            if self.drive_handle is None and task.get("goal_pose_mm") is not None:
                # Start navigation
                print(f"Driving toward: {task.get('visualTarget')}...")

                self.drive_handle = self.robot.move_to(
                    x: x,
                    y: y,
                    velocity: float,
                    tolerance: TOLERANCE_MM,
                    blocking: bool = False,
                    timeout: float = None
                )
                # TODO: Actually, non-blocking continue on trajectory
            
            if self.drive_handle.is_done():
                # TODO: Add go to heading
                print(f"{task.get('goal_pose_mm')} reached")
                if DEBUG_MODE:
                    # Simulate visual acquisition via Button 1
                    if self.robot.was_button_pressed(Button.BTN_1):
                        self.trigger("next")
                # TODO: Add camera subscription comparison to check

        elif task_type == "MANIP":
            cmd = task.get("cmd")
            if cmd == "pick" or cmd == "place":
                if self.timer_start is None:
                    self.timer_start = time.monotonic()
                    print(f"Manipulator action: {task.get('cmd')}...")
                
            else:
                # Invalid command
                self.trigger("error")
            
            if time.monotonic() - self.timer_start > 1.5:
                self.trigger("next")

        elif task_type == "IDENT":
            if self.timer_start is None:
                self.timer_start = time.monotonic()
                print("Processing visual identification...")
            
            if time.monotonic() - self.timer_start > TIMEOUT_CAM_DETECTION:
                # TODO: Fallback instead, but error for now
                self.trigger("error")

        elif task_type == "DONE":
            self.robot.stop()
            # Final state; no further transitions


def run(robot: Robot) -> None:
    configure_robot(robot)
    start_robot(robot)

    # TODO: Home stepper
    # TODO: Home servo
    # TODO: Validate sensors
    
    fsm = MissionFSM(robot, tasks)
    fsm.spin() # Blocks until the program is terminated
