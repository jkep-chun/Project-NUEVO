from __future__ import annotations

import math
import time
import os
import numpy as np

from robot.fsm_helpers.path_helpers import generate_open_rounded_path

from robot.robot import Robot
from robot.hardware_map import (
    Button, 
    LIFT_STEPPER, 
    LIFT_EXTEND_STEPS, 
    LIFT_LOWER_STEPS, 
    LIFT_BUFFER_STEPS,
    GRIPPER_SERVO,
    GRIPPER_OPEN_DEG,
    GRIPPER_CLOSE_DEG,
    StepMoveType,
    DEFAULT_FSM_HZ
)
from robot.fsm_helpers.burgerbot_parameters import (
    NavStage, 
    ManipStage,
    PICK_SEQUENCE, 
    PLACE_SEQUENCE,
    TOLERANCE_MM, 
    VELOCITY_MM_S, 
    LOOKAHEAD_MM,
    ADVANCE_RADIUS_MM
)
from robot.fsm_helpers.vision_helpers import find_traffic_light_color
from robot.fsm_helpers.firmware_helpers import StepHomingHandle

ENABLE_CAM = False

class Task:
    def __init__(self, robot: Robot):
        self.robot = robot

    def update(self) -> None:
        pass

    def is_done(self) -> bool:
        return True
    
    def on_enter(self) -> None:
        pass

    def on_exit(self) -> None:
        pass


class NavTask(Task):
    def __init__(self, robot, waypoints, goal_heading=None, path_planner="pp"):
        super().__init__(robot)
        # Ensure waypoints is a list of tuples
        if waypoints is not None:
            self.waypoints = [tuple(wp) for wp in np.atleast_2d(waypoints)]
        else:
            self.waypoints = []
        
        self.goal_heading = goal_heading
        self.path_planner = path_planner

        self._is_done = False
        self.wp_lapf_idx = 0
        self.handle = None
        self.start_heading = None

        self.stage_idx = 0
        self.stage_sequence = []

        if self.waypoints:
            self.stage_sequence.append(NavStage.POSITION)
            # Add an initial heading pivot if in purepursuit
            if self.path_planner == "pp":
                x, y, _ = self.robot.get_pose()
                dx = self.waypoints[0][0] - x
                dy = self.waypoints[0][1] - y
                if math.hypot(dx, dy) > TOLERANCE_MM:
                    self.start_heading = math.degrees(math.atan2(dy, dx))
                    self.stage_sequence.insert(0, NavStage.START_HEADING)   

        # Add goal heading if provided
        if self.goal_heading is not None:
            self.stage_sequence.append(NavStage.HEADING)

        if self.stage_sequence:
            # Set stage sequence
            self.stage = self.stage_sequence[self.stage_idx]
        else:
            self.stage = None
            self._is_done = True

    def update(self):

        if self.handle and self.handle.is_done():
            self.handle = None
            if self.stage == NavStage.POSITION and self.path_planner == "lapf" and self.wp_lapf_idx < len(self.waypoints) - 1:
                # Stay in NavStage.POSITION
                self.wp_lapf_idx += 1
            else:
                self.stage_idx += 1
                if self.stage_idx < len(self.stage_sequence):
                    self.stage = self.stage_sequence[self.stage_idx]
                else:
                    self._is_done = True
                    return

        if self.stage == NavStage.START_HEADING:
            if self.handle is None:
                print(f"Turning to face first waypoint: {self.start_heading:.2f} deg")
                self.handle = self.robot.turn_to(
                    angle_deg=self.start_heading,
                    blocking=False,
                    tolerance_deg=5.0,
                    timeout=None
                )

        elif self.stage == NavStage.POSITION:
            if self.handle is None:
                if self.path_planner == "pp":
                    print(f"Driving seg with {len(self.waypoints)} waypoints")
                    self.handle = self.robot.purepursuit_follow_path(
                        waypoints=self.waypoints,
                        velocity=VELOCITY_MM_S,
                        lookahead=LOOKAHEAD_MM,
                        tolerance=TOLERANCE_MM,
                        blocking=False,
                        max_angular_rad_s=1.0,
                        advance_radius=ADVANCE_RADIUS_MM
                    )
                elif self.path_planner == "lapf":
                    wp = self.waypoints[self.wp_lapf_idx]
                    tx, ty = wp[0], wp[1]
                    print(f"Driving (LAPF) toward: ({tx:.2f},{ty:.2f})")
                    self.handle = self.robot.lapf_to_goal(
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

        elif self.stage == NavStage.HEADING:
            if self.handle is None:
                print(f"Turning to: {self.goal_heading:.2f} deg")
                self.handle = self.robot.turn_to(
                    angle_deg=self.goal_heading,
                    blocking=False,
                    tolerance_deg=2.0,
                    timeout=None
                )

    def is_done(self) -> bool:
        return self._is_done

    def on_exit(self) -> None:
        """
        Save a plot of the robot's trajectory and waypoints to a .jpg file
        in /runtime_output/plots/ upon exiting the navigation task.
        """
        try:
            import matplotlib
            matplotlib.use('Agg') # Headless-safe
            import matplotlib.pyplot as plt
        except ImportError:
            print("[NavTask] matplotlib not installed; skipping path plot")
            return

        # Trajectory data from Robot (mm)
        odom = list(self.robot._odom_traj)
        fused = list(self.robot._fused_traj)
        
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

        # Plot current task waypoints
        if self.waypoints:
            wx = [wp[0] for wp in self.waypoints]
            wy = [wp[1] for wp in self.waypoints]
            plt.scatter(wx, wy, color='green', marker='o', s=40, label='Task Waypoints', zorder=5)
            plt.plot(wx, wy, color='green', linestyle='--', alpha=0.2)
            
            if len(wx) > 0:
                plt.text(wx[0], wy[0], ' Start', color='green', verticalalignment='bottom', fontsize=9)
                plt.text(wx[-1], wy[-1], ' End', color='red', verticalalignment='bottom', fontsize=9)

        # Current robot pose
        curr_x, curr_y, _ = self.robot.get_pose()
        plt.scatter([curr_x], [curr_y], color='red', marker='X', s=100, label='Final Position', zorder=10)

        plt.xlabel('X (mm)')
        plt.ylabel('Y (mm)')
        plt.title(f'Robot Navigation Trajectory ({self.path_planner.upper()})')
        plt.legend()
        plt.axis('equal')
        plt.grid(True, linestyle=':', alpha=0.6)
        
        # Determine save directory (prioritize Docker /runtime_output)
        save_dir = "/runtime_output/plots"
        if not os.path.exists(save_dir):
            # Fallback to local project structure
            save_dir = "ros2_ws/runtime_output/plots"
            
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, "final_trajectory.jpg")
        
        try:
            plt.savefig(save_path, format='jpg', dpi=150)
            print(f"[NavTask] Trajectory plot saved to {save_path}")
        except Exception as e:
            print(f"[NavTask] Error saving plot: {e}")
        finally:
            plt.close()


class WaitTask(Task):
    def __init__(self, robot, params: dict):
        super().__init__(robot)
        self._is_done = False
        self._params = params

    def update(self):
        if self._params.get("trigger") == "green_light":
            if not ENABLE_CAM:
                if self.robot.was_button_pressed(Button.BTN_1):
                    print("trigger - BTN_1 (debug)")
                    self._is_done = True
            elif find_traffic_light_color(self.robot) == "green":
                print("trigger - green light detected")
                self._is_done = True

    def is_done(self) -> bool:
        return self._is_done


class ManipTask(Task):
    def __init__(self, robot, params: dict):
        super().__init__(robot)
        self._is_done = False
        cmd = params.get("cmd")
        if cmd == "pick":
            self.sequence = PICK_SEQUENCE
        elif cmd == "place":
            self.sequence = PLACE_SEQUENCE
        else:
            self.sequence = []
            self._is_done = True
        
        self.idx = 0
        self.handle = None
        self.stage_init = True
        self.timer_start = None

    def update(self):
        if self._is_done:
            return

        stage = self.sequence[self.idx]

        if self.stage_init:
            print(f"ManipStage: {stage.name}")
            if stage == ManipStage.LEVEL:
                self.robot.step_enable(LIFT_STEPPER)
                self.robot.step_move(LIFT_STEPPER, LIFT_EXTEND_STEPS, StepMoveType.ABSOLUTE, blocking=False)
                self.handle = StepHomingHandle(self.robot, LIFT_STEPPER) # Reuse StepHomingHandle for non-blocking move
            elif stage == ManipStage.OPEN:
                self.robot.enable_servo(GRIPPER_SERVO)
                self.robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_DEG)
                self.timer_start = time.monotonic()
            elif stage == ManipStage.CLOSE:
                self.robot.enable_servo(GRIPPER_SERVO)
                self.robot.set_servo(GRIPPER_SERVO, GRIPPER_CLOSE_DEG)
                self.timer_start = time.monotonic()
            elif stage == ManipStage.FORWARD:
                self.handle = self.robot.move_forward(distance=100.0, velocity=VELOCITY_MM_S, tolerance=TOLERANCE_MM, blocking=False)
            elif stage == ManipStage.RETREAT:
                self.handle = self.robot.move_backward(distance=100.0, velocity=VELOCITY_MM_S, tolerance=TOLERANCE_MM, blocking=False)
            elif stage == ManipStage.LOWER:
                self.robot.step_enable(LIFT_STEPPER)
                self.robot.step_move(LIFT_STEPPER, LIFT_LOWER_STEPS, StepMoveType.RELATIVE, blocking=False)
                self.handle = StepHomingHandle(self.robot, LIFT_STEPPER)
            elif stage == ManipStage.RAISE:
                self.robot.step_enable(LIFT_STEPPER)
                self.robot.step_move(LIFT_STEPPER, LIFT_BUFFER_STEPS, StepMoveType.RELATIVE, blocking=False)
                self.handle = StepHomingHandle(self.robot, LIFT_STEPPER)
            
            self.stage_init = False

        # Check for completion
        done = False
        if self.handle:
            if self.handle.is_done():
                self.handle = None
                done = True
        elif self.timer_start:
            if time.monotonic() - self.timer_start > 1.0: # 1s settle for servo
                self.timer_start = None
                done = True
        
        if done:
            self.idx += 1
            if self.idx < len(self.sequence):
                self.stage_init = True
            else:
                self._is_done = True

    def is_done(self) -> bool:
        return self._is_done


class IdentTask(Task):
    def __init__(self, robot):
        super().__init__(robot)
        self._is_done = False
        self.timer_start = None

    def update(self):
        if self.timer_start is None:
            print("IdentTask: Processing...")
            self.timer_start = time.monotonic()
        
        if time.monotonic() - self.timer_start > 2.0:
            self._is_done = True

    def is_done(self) -> bool:
        return self._is_done


class PlanTask(Task):
    def __init__(self, robot):
        super().__init__(robot)
        self._is_done = False
        self.timer_start = None

    def update(self):
        if self.timer_start is None:
            print("PlanTask: Planning...")
            self.timer_start = time.monotonic()
        
        if time.monotonic() - self.timer_start > 1.0:
            self._is_done = True

    def is_done(self) -> bool:
        return self._is_done


def build_task(robot: Robot, task_dict: dict) -> Task:
    state = task_dict.get("state")
    if state == "NAV":
        return NavTask(
            robot=robot,
            waypoints=generate_open_rounded_path(
                vertices=task_dict.get("waypoints"),
                R=100.0,
                ds=25.0
            ),
            goal_heading=task_dict.get("goal_heading"),
            path_planner=task_dict.get("path_planner", "pp") # Defaults to "pp" over None
        )
    elif state == "WAIT":
        return WaitTask(robot, task_dict)
    elif state == "MANIP":
        return ManipTask(robot, task_dict)
    elif state == "IDENT":
        return IdentTask(robot)
    elif state == "PLAN":
        return PlanTask(robot)
    else:
        return Task(robot)
