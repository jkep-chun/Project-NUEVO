from __future__ import annotations

import math
import time

from robot.robot import Robot
from robot.util import densify_polyline
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
    LOOKAHEAD_MM
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
    def __init__(self, robot, waypoints, path_planner="pp"):
        super().__init__(robot)
        # Ensure waypoints is a list of tuples/lists
        if waypoints and not isinstance(waypoints[0], (list, tuple)):
            waypoints = [waypoints]
        self.waypoints = waypoints
        self.path_planner = path_planner

        self._is_done = False

        self.wp_idx_curr = 0
        self.wp_idx_last = 0

        self.handle = None

        self.seg_init = True
        self.seg_waypoints = []
        self.seg_start_θ = None
        self.seg_heading = None

        self.stage = None
        self.stage_idx = 0
        self.stage_sequence = []

    def update(self):
        if self.seg_init:
            seg_idx_start = self.wp_idx_curr
            self.seg_waypoints.clear()
            self.seg_heading = None
            self.stage_sequence.clear()
            self.stage_sequence.append(NavStage.POSITION)

            while self.wp_idx_curr < len(self.waypoints):
                wp = self.waypoints[self.wp_idx_curr]
                wp_x, wp_y, wp_θ = wp[0], wp[1], wp[2:]
                self.seg_waypoints.append((wp_x, wp_y))

                if wp_θ:
                    self.seg_heading = float(wp_θ[0])
                    self.stage_sequence.append(NavStage.HEADING)
                    break

                if self.wp_idx_curr < len(self.waypoints) - 1:
                    self.wp_idx_curr += 1
                else:
                    break
            
            self.wp_idx_last = self.wp_idx_curr

            # Add an initial heading pivot if in purepursuit
            if self.path_planner == "pp":
                # Densify segment for pure pursuit
                self.seg_waypoints = densify_polyline(self.seg_waypoints, spacing=50.0)
                
                x, y, _ = self.robot.get_pose()
                dx = self.seg_waypoints[0][0] - x
                dy = self.seg_waypoints[0][1] - y
                if math.hypot(dx, dy) > 10.0:
                    self.seg_start_θ = math.degrees(math.atan2(dy, dx))
                    self.stage_sequence.insert(0, NavStage.START_HEADING)
            elif self.path_planner == "lapf":
                self.wp_idx_curr = seg_idx_start

            # Reset for new stage sequence
            self.stage_idx = 0
            self.stage = self.stage_sequence[self.stage_idx]
            self.handle = None
            self.seg_init = False

        if self.handle and self.handle.is_done():
            self.handle = None
            if self.stage == NavStage.POSITION and self.path_planner == "lapf" and self.wp_idx_curr < self.wp_idx_last:
                self.wp_idx_curr += 1
                # Stay in POSITION stage, handle will be re-initialized below
            else:
                self.stage_idx += 1
                if self.stage_idx < len(self.stage_sequence):
                    self.stage = self.stage_sequence[self.stage_idx]
                else:
                    self.wp_idx_curr = self.wp_idx_last + 1
                    
                    if self.wp_idx_curr < len(self.waypoints):
                        self.seg_init = True
                    else:
                        self._is_done = True
                        return

        if self.stage == NavStage.START_HEADING:
            if self.handle is None:
                print(f"Turning to face first waypoint: {self.seg_start_θ:.2f} deg")
                self.handle = self.robot.turn_to(
                    angle_deg=self.seg_start_θ,
                    blocking=False,
                    tolerance_deg=5.0,
                    timeout=None
                )

        elif self.stage == NavStage.POSITION:
            if self.handle is None:
                if self.path_planner == "pp":
                    print(f"Driving seg with {len(self.seg_waypoints)} waypoints")
                    self.handle = self.robot.purepursuit_follow_path(
                        waypoints=self.seg_waypoints,
                        velocity=VELOCITY_MM_S,
                        lookahead=LOOKAHEAD_MM,
                        tolerance=TOLERANCE_MM,
                        blocking=False,
                        max_angular_rad_s=1.0,
                        advance_radius=50.0
                    )
                elif self.path_planner == "lapf":
                    wp = self.waypoints[self.wp_idx_curr]
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
                print(f"Turning to: {self.seg_heading:.2f} deg")
                self.handle = self.robot.turn_to(
                    angle_deg=self.seg_heading,
                    blocking=False,
                    tolerance_deg=2.0,
                    timeout=None
                )

    def is_done(self) -> bool:
        return self._is_done


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
        return NavTask(robot, task_dict.get("waypoints"), task_dict.get("path_planner", "pp"))
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
