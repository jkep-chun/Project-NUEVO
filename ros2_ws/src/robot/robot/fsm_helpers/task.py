from __future__ import annotations

import math

from robot.robot import Robot
from robot.util import densify_polyline
from robot.hardware_map import Button
from robot.fsm_helpers.burgerbot_parameters import NavStage, TOLERANCE_MM, VELOCITY_MM_S, LOOKAHEAD_MM
from robot.fsm_helpers.vision_helpers import find_traffic_light_color

ENABLE_CAM = False

class Task:
    def __init__(self, robot: Robot):
        self.robot = robot

    def update(self) -> None:
        pass

    def is_done(self) -> bool:
        return True


class NavTask(Task):
    def __init__(self, robot, waypoints, path_planner):
        super().__init__(robot)
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

        self.stage = []
        self.stage_idx = 0
        self.stage_sequence = []

    def update(self):
        if self.seg_init:
            seg_idx_start = self.wp_idx_curr
            self.seg_waypoints.clear()
            self.seg_heading = []
            self.stage_sequence.clear()
            self.stage_sequence.append(NavStage.POSITION)

            while self.wp_idx_curr < len(self.waypoints):
                wp = self.waypoints[self.wp_idx_curr]
                wp_x, wp_y, wp_θ = wp[0], wp[1], wp[2:]
                self.seg_waypoints.append((wp_x, wp_y))

                if wp_θ:
                    self.seg_heading = wp_θ
                    self.stage_sequence.append(NavStage.HEADING)
                    break

                if self.wp_idx_curr < len(self.wp_idx_curr) - 1:
                    self.wp_idx_curr += 1
                else:
                    break
            
            self.wp_idx_last = self.wp_idx_curr

            # Add an initial heading pivot if in purepursuit
            if self.path_planner == "pp":
                x, y, _ = self.robot.get_pose()
                dx = self.seg_waypoints[0][0] - x
                dy = self.seg_waypoints[0][1] - y
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
            else:
                self.stage_idx += 1
                if self.stage_idx < len(self.stage_sequence):
                    self.stage = self.stage_sequence[self.stage_idx]
                else:
                    self.wp_idx = self.wp_idx_last + 1
                    
                    if self.wp_idx < len(self.waypoints):
                        self.nav_init = True
                    else:
                        self._is_done = True

        if self.stage == NavStage.START_HEADING:
            if self.handle is None:
                print(f"Turning to face first waypoint: {self.seg_start_θ:.2f} deg")
                self.handle = self.robot.turn_to(
                    angle_deg=self.seg_start_θ,
                    blocking=False,
                    tolerance_deg=5.0,
                    timeout=None
                )
                return

        elif self.stage == NavStage.POSITION:
            if self.handle is None:
                if self.path_planner == "pp":
                    print(f"Driving seg with {len(self.seg_waypoints)} waypoints")
                    self.handle = self.robot.purepursuit_follow_path(
                        waypoints=densify_polyline(self.seg_waypoints, spacing=50.0),
                        velocity=VELOCITY_MM_S,
                        lookahead=LOOKAHEAD_MM,
                        tolerance=TOLERANCE_MM,
                        blocking=False,
                        max_angular_rad_s=1.0,
                        advance_radius=50.0
                    )
                elif self.path_planner == "lapf":
                    wp = self.seg_waypoints[self.wp_idx_curr]
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
                return

        elif self.stage == NavStage.HEADING:
            if self.handle is None:
                print(f"Turning to: {self.nav_target_theta:.2f} deg")
                self.handle = self.robot.turn_to(
                    angle_deg=self.nav_target_theta,
                    blocking=False,
                    tolerance_deg=2.0,
                    timeout=None
                )
                return

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
    def __init__(self, robot):
        super().__init__(robot)
        self._is_done = False

    def update(self):

    def is_done(self) -> bool:
        return self._is_done