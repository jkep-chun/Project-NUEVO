from __future__ import annotations
import time, sqlite3, os
from datetime import datetime

from robot.robot import Robot
from robot.hardware_map import Button
from robot.robot_fsm import RobotFSM

# TODO: Change path when complete (MED)
import robot.fsm_helpers.firmware_helpers as fh
from robot.fsm_helpers.task_planner import tasks
from robot.fsm_helpers.task import build_task, Task

LOG_DIR = "/runtime_output/logs"

class MissionFSM(RobotFSM):
    def __init__(self, robot: Robot, task_list: list[dict]):
        super().__init__(robot, initial_state_str="INIT")

        # Robust logging path
        self._log_dir = LOG_DIR
        if not os.path.exists(self._log_dir):
            self._log_dir = "ros2_ws/runtime_output/logs"

        # Tasks
        self.tasks = task_list
        self.current_task: Task = None
        self.task_idx = 0
        self.mission_data = {
            "waypoints": [],
            "vertices": [],
            "burger_stack": [],
            "delivery_status": False,
            "matched_customer": None
        }

        # Motion handles (mostly managed by tasks now, but kept for homing)
        self.homing_lift_handle = None
        self.homing_gripper_handle = None

        # Execution
        self.timer_start = None
        self.execution_print_period = 1.0
        self._start_traj_idx = 0
        
        # Transitions
        self.add_transition("INIT", "to_execute", "EXECUTE")
        self.add_transition("INIT", "to_homing", "HOMING")
        self.add_transition("HOMING", "to_init", "INIT")
        self.add_transition("EXECUTE", "next", "EXECUTE", action=self._advance_task)
        self.add_transition("EXECUTE", "e-stop", "ERROR")
        self.add_transition("EXECUTE", "to_done", "DONE")
        self.add_transition("EXECUTE", "error", "ERROR")
        self.add_transition("DONE", "to_init", "INIT")

        # SQLITE3 LOGGING SETUP
        self.conn = None
        self._setup_logging()

    def _setup_logging(self) -> None:
        """Initialize sqlite3 database for logging."""
        if self.conn:
            self.conn.close()
            self.conn = None

        os.makedirs(self._log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        db_path = os.path.join(self._log_dir, f"fsm_log_{timestamp}.db")
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
            fh.start_robot(self.robot)
            fh.reset_mission_pose(self.robot)
            print(
                "\n>>> INIT - FIRMWARE RUNNING" \
                "\n    BTN_1 -> EXECUTE" \
                "\n    BTN_2 -> HOMING"
            )

        elif state == "HOMING":
            self.homing_lift_handle = fh.home_lift(self.robot)
            self.homing_gripper_handle = fh.home_gripper(self.robot)
            print("\n>>> HOMING - BEGIN SEQUENCE")

        elif state == "EXECUTE":
            self.timer_start = time.monotonic() # Reset timer for the new task

            if self.task_idx == 0:
                self._start_traj_idx = len(self.robot._fused_traj)

            if self.task_idx < len(self.tasks):
                task_dict = self.tasks[self.task_idx]
                self.current_task = build_task(self.robot, task_dict, self.mission_data)
                self.current_task.on_enter()
                print(f"\n>>> EXECUTE - TASK {self.task_idx}: {self.tasks[self.task_idx]}")
            else:
                self.trigger("to_done")

        elif state == "DONE":
            self._plot_trajectory()
            self.robot.shutdown()
            if self.conn:
                self.conn.close()
                self.conn = None
            print(
                "\n>>> DONE - TASKS COMPLETE" \
                "\n    BTN_1 -> INIT"
            )

        elif state =="ERROR":
            self.robot.shutdown()
            if self.conn:
                self.conn.close()
                self.conn = None
            self.robot.estop()
            print("\n>>> ERROR (CTRL + C TO TERMINATE NODE)")


    def _plot_trajectory(self) -> None:
        """
        Save a plot of the robot's trajectory and waypoints to a .jpg file
        in /runtime_output/plots/ upon completing the mission.
        """
        try:
            import matplotlib
            matplotlib.use('Agg') # Headless-safe
            import matplotlib.pyplot as plt
        except ImportError:
            print("[MissionFSM] matplotlib not installed; skipping path plot")
            return

        # Trajectory data from Robot (mm)
        odom = list(self.robot._odom_traj)[self._start_traj_idx:]
        fused = list(self.robot._fused_traj)[self._start_traj_idx:]
        
        if not odom and not fused:
            return

        plt.figure(figsize=(10, 8))
        
        # Plot raw odometry (light blue)
        if odom:
            ox, oy = zip(*odom)
            plt.plot(ox, oy, label='Odometry (Raw)', color='steelblue', alpha=0.4, linewidth=1)
        
        # Plot fused trajectory (bold orange)
        if fused:
            fx, fy = zip(*fused)
            plt.plot(fx, fy, label='Fused Trajectory', color='darkorange', linewidth=2)

        # Plot all mission waypoints
        waypoints = self.mission_data.get("waypoints", [])
        if waypoints:
            wx = [wp[0] for wp in waypoints]
            wy = [wp[1] for wp in waypoints]
            plt.scatter(wx, wy, color='green', marker='o', s=40, label='Waypoint', zorder=5)
            
            if len(wx) > 0:
                plt.text(wx[0], wy[0], ' Start', color='green', verticalalignment='bottom', fontsize=9)
                plt.text(wx[-1], wy[-1], ' End', color='red', verticalalignment='bottom', fontsize=9)

        # Plot all mission vertices
        vertices = self.mission_data.get("vertices", [])
        if vertices:
            vx = [v[0] for v in vertices]
            vy = [v[1] for v in vertices]
            plt.scatter(vx, vy, color='purple', marker='o', s=65, label='Vertices', zorder=4)

        # Current robot pose
        curr_x, curr_y, _ = self.robot.get_pose()
        plt.scatter([curr_x], [curr_y], color='red', marker='X', s=100, label='Final Position', zorder=10)

        plt.xlabel('X (mm)')
        plt.ylabel('Y (mm)')
        plt.title('Robot Navigation Trajectory (Mission)')
        plt.legend()
        plt.axis('equal')
        plt.grid(True)
        
        # Determine save directory
        save_dir = "/runtime_output/plots"
        if not os.path.exists(save_dir):
            save_dir = "ros2_ws/runtime_output/plots"
            
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, "final_trajectory.jpg")
        
        try:
            plt.savefig(save_path, format='jpg', dpi=150)
            print(f"[MissionFSM] Trajectory plot saved to {save_path}")
        except Exception as e:
            print(f"[MissionFSM] Error saving plot: {e}")
        finally:
            plt.close()


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
            # BTN_1 skip check
            if self.robot.was_button_pressed(Button.BTN_1):
                if self.current_task:
                    print(f"[{state_str}] Skipping task {self.task_idx} via BTN_1")
                    self.current_task.cancel()
                self.trigger("next")
                return

            # E-STOP check
            if self.robot.was_button_pressed(Button.BTN_2):
                print(f"[{state_str}] Emergency stop via BTN_2")
                self.trigger("e-stop")
                return

            # Proceed normally
            if self.current_task:
                self.current_task.update()
                if self.current_task.is_done():
                    self.trigger("next")

        elif state_str == "DONE":
            if self.robot.was_button_pressed(Button.BTN_1):
                self._setup_logging()
                self.trigger("to_init")

        elif state_str == "ERROR":
            pass


def run(robot: Robot) -> None:
    fh.configure_robot(robot)
    fsm = MissionFSM(robot, tasks)
    fsm.on_enter("INIT")
    fsm.spin()
