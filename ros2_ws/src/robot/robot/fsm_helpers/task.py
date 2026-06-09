from __future__ import annotations

import math
import time
import numpy as np

from robot.fsm_helpers.path_helpers import generate_open_rounded_path
from robot.util import densify_polyline

from robot.robot import Robot
import robot.hardware_map as hm
import robot.fsm_helpers.burgerbot_parameters as bp
import robot.fsm_helpers.course_parameters as cp
import robot.fsm_helpers.vision_helpers as vh
import robot.fsm_helpers.firmware_helpers as fh
import robot.fsm_helpers.ingredient_helpers as ih

ENABLE_CAM = True

class Task:
    def __init__(self, robot: Robot, mission_data: dict, params: dict):
        self._robot = robot
        self._mission_data = mission_data
        self._params = params

    def update(self) -> None:
        pass

    def is_done(self) -> bool:
        return True
    
    def on_enter(self) -> None:
        pass

    def on_exit(self) -> None:
        pass

    def cancel(self) -> None:
        """Abort the task and stop relevant hardware."""
        self._robot.stop()


class HomeTask(Task):
    """Homing has two physical components: the lift (stepper) and gripper (servo), and their sequences are handled in parallel"""
    
    def __init__(self, robot: Robot, mission_data: dict, params: dict):
        super().__init__(robot, mission_data, params)
        self._is_done = False
        self._lift_handle = None
        self._gripper_handle = None
        self._lift_done = False
        self._gripper_done = False
        self._home_stages = [bp.HomeStage.HOME]
        self._home_stage_idx = 0

        if self._params.get("lift_init_height"):
            self._home_stages.append(bp.HomeStage.INIT)
            self._lift_init_height = self._params["lift_init_height"]

    def on_enter(self) -> None:
        print("[HomeTask] ENTER: Homing lift and gripper.")
        
        # Start lift homing only if not already there
        if self._robot.get_limit(hm.Limit.LIM_1):
            print("[HomeTask] Lift already at home limit.")
            if self._home_stage_idx < len(self._home_stages) - 1:
                self._home_stage_idx += 1
            else:
                self._lift_done = True
        else:
            self._lift_handle = fh.home_lift(self._robot)
        
        # 2. Start gripper homing only if not already there
        if self._robot.get_limit(hm.GRIPPER_LIMIT):
            print("[HomeTask] Gripper already at home limit.")
            self._gripper_done = True
        else:
            self._gripper_handle = fh.home_gripper(self._robot)

    def update(self) -> None:
        home_stage = self._home_stages[self._home_stage_idx]

        if self._is_done:
            return

        if not self._lift_done:
            if home_stage == bp.HomeStage.HOME:
                if self._robot.get_limit(hm.Limit.LIM_1) or (self._lift_handle and self._lift_handle.is_done()):
                    print(f"[HomeTask] Lift homing complete.")
                    if self._lift_handle:
                        self._lift_handle.cancel()
                        self._lift_handle = None
                    if self._home_stage_idx < len(self._home_stages) - 1:
                        self._home_stage_idx += 1
                    else:
                        self._lift_done = True
            elif home_stage == bp.HomeStage.INIT:
                if self._lift_handle is None:
                    print(f"[HomeTask] Lift initialization commencing.")
                    self._lift_handle  = fh.move_lift(
                            robot=self._robot,
                            position=self._lift_init_height,
                            move_type=hm.StepMoveType.ABSOLUTE,
                            disable_on_done=False
                        )
                elif self._lift_handle and self._lift_handle.is_done():
                    print(f"[HomeTask] Lift init complete.")
                    if self._lift_handle:
                        self._lift_handle.cancel()
                        self._lift_handle = None
                    if self._home_stage_idx < len(self._home_stages) - 1:
                        self._home_stage_idx += 1
                    else: self._lift_done = True
        
        # Check gripper homing progress
        if not self._gripper_done:
            if self._robot.get_limit(hm.GRIPPER_LIMIT) or (self._gripper_handle and self._gripper_handle.is_done()):
                print(f"[HomeTask] Gripper homing complete.")
                if self._gripper_handle:
                    self._gripper_handle.cancel()
                    self._gripper_handle = None
                self._gripper_done = True

        # Task is done when both subsystems have reached home
        if self._lift_done and self._gripper_done:
            self._is_done = True

    def cancel(self) -> None:
        if self._lift_handle:
            self._lift_handle.cancel()
            self._lift_handle = None
        if self._gripper_handle:
            self._gripper_handle.cancel()
            self._gripper_handle = None
        super().cancel()

    def is_done(self) -> bool:
        return self._is_done

    def on_exit(self) -> None:
        print("[HomeTask] EXIT: Homing complete.")


class NavTask(Task):
    def __init__(self, robot: Robot, mission_data: dict, params: dict):
        super().__init__(robot, mission_data, params)
        
        # 1. Extract parameters
        waypoints = list(self._params.get("waypoints", []))
        self._goal_heading = self._params.get("goal_heading")
        self._path_planner = self._params.get("path_planner", "pp")
        self._delivery_segment = False
        self._enable_slam_localization = self._params.get("enable_slam_localization", False)
        self._slam_position_fusion_alpha = self._params.get("slam_position_fusion_alpha", bp.SLAM_POSITION_FUSION_ALPHA)
        self._slam_orientation_fusion_alpha = self._params.get("slam_orientation_fusion_alpha", bp.SLAM_ORIENTATION_FUSION_ALPHA)

        # Profiling parameters
        self._use_profile = self._params.get("use_profile", False)
        self._accel = self._params.get("accel", None)
        self._decel = self._params.get("decel", self._accel)
        self._angular_accel = self._params.get("angular_accel", None)
        self._angular_decel = self._params.get("angular_decel", self._angular_accel)

        if self._path_planner == "lapf":
            self._velocity = self._params.get("velocity", bp.VELOCITY_LAPF)
            self._max_angular_rad_s = self._params.get("max_angular_rad_s", bp.ANGULAR_RAD_S_LAPF)
        else:
            self._velocity = self._params.get("velocity", bp.VELOCITY_MM_S)
            self._max_angular_rad_s = self._params.get("max_angular_rad_s", 1.0)

        # 2. Robustness: Inject customer data BEFORE path generation if mission requires it
        if not self._mission_data.get("delivery_status"):
            matched_customer = self._mission_data.get("matched_customer")
            if matched_customer == 1:
                waypoints.append(cp.WP_CUSTOMER_1)
                self._goal_heading = 0
                self._delivery_segment = True
            elif matched_customer == 2:
                waypoints.append(cp.WP_CUSTOMER_2)
                self._goal_heading = 0
                self._delivery_segment = True

        # 3. Path Processing
        if waypoints:
            self._vertices = [tuple(wp) for wp in np.atleast_2d(waypoints)]
        else:
            self._vertices = []
        
        if self._enable_slam_localization:
            self._robot.enable_slam_localization()
        else:
            self._robot.disable_slam_localization()
        self._robot.set_slam_position_fusion_alpha(self._slam_position_fusion_alpha)
        self._robot.set_slam_orientation_fusion_alpha(self._slam_orientation_fusion_alpha)

        self._is_done = False
        self._wp_lapf_idx = 0
        self._handle = None
        self._start_heading = None
        self._start_traj_idx = len(self._robot._fused_traj)

        self._stage_idx = 0
        self._stage_sequence = []
        self._waypoints = list(self._vertices) # Fallback to original vertices

        if self._vertices:
            self._stage_sequence.append(bp.NavStage.POSITION)
            # Remove vertices if within TOLERANCE_MM of initial pose
            x, y, _ = self._robot.get_pose()
            while self._vertices:
                v = self._vertices[0]
                dx = v[0] - x
                dy = v[1] - y
                if math.hypot(dx, dy) < bp.TOLERANCE_MM:
                    self._vertices.pop(0)
                else:
                    break
            path_vertices = [(x, y)] + self._vertices

            if self._path_planner == "pp":
                # Make a dense rounded path if in pure pursuit
                self._waypoints = generate_open_rounded_path(
                    vertices=path_vertices,
                    R=bp.TURN_RADIUS,
                    ds=bp.WP_SPACING
                )

            elif self._path_planner == "lapf":
                # Make denser polyline path if in lapf
                self._waypoints = densify_polyline(
                    control_points=path_vertices,
                    spacing=bp.WP_SPACING_LAPF
                )
                
            # Add an initial heading pivot
            for wp in self._waypoints:
                dx = wp[0] - x
                dy = wp[1] - y
                if math.hypot(dx, dy) > bp.TOLERANCE_MM:
                    self._start_heading = math.degrees(math.atan2(dy, dx))
                    self._stage_sequence.insert(0, bp.NavStage.START_HEADING)
                    break
        
        # Add goal heading if provided
        if self._goal_heading is not None:
            self._stage_sequence.append(bp.NavStage.HEADING)

        if self._stage_sequence:
            # Set stage sequence
            self._stage = self._stage_sequence[self._stage_idx]
        else:
            self._stage = None
            self._is_done = True

    def update(self):
        if self._is_done:
            return

        if self._handle and self._handle.is_done():
            self._handle = None
            self._stage_idx += 1
            if self._stage_idx < len(self._stage_sequence):
                self._stage = self._stage_sequence[self._stage_idx]
            else:
                if self._delivery_segment:
                    self._mission_data["delivery_status"] = True
                self._is_done = True
                return

        if self._stage == bp.NavStage.START_HEADING:
            if self._handle is None:
                print(f"Turning to face first waypoint: {self._start_heading:.2f} deg")
                self._handle = self._robot.turn_to(
                    angle_deg=self._start_heading,
                    blocking=False,
                    tolerance_deg=5.0,
                    timeout=None,
                    use_profile=self._use_profile,
                    accel=self._angular_accel,
                    decel=self._angular_decel
                )

        elif self._stage == bp.NavStage.POSITION:
            if self._handle is None:
                if self._path_planner == "pp":
                    print(f"Driving seg with {len(self._waypoints)} waypoints")
                    self._handle = self._robot.purepursuit_follow_path(
                        waypoints=self._waypoints,
                        velocity=self._velocity,
                        lookahead=bp.LOOKAHEAD_MM,
                        tolerance=bp.TOLERANCE_MM,
                        blocking=False,
                        max_angular_rad_s=self._max_angular_rad_s,
                        advance_radius=bp.ADVANCE_RADIUS_MM,
                        use_profile=self._use_profile,
                        accel=self._accel,
                        decel=self._decel
                    )
                elif self._path_planner == "lapf":
                    print(f"Driving (LAPF) seg with {len(self._waypoints)} waypoints")
                    self._handle = self._robot.lapf_follow_path(
                        waypoints=self._waypoints,
                        velocity=self._velocity,
                        max_angular_rad_s=self._max_angular_rad_s,
                        tolerance=bp.TOLERANCE_LAPF,
                        repulsion_range=bp.REPULSION_RANGE_LAPF,
                        repulsion_gain=bp.REPULSION_GAIN_LAPF,
                        attraction_gain=bp.ATTRACTION_GAIN_LAPF,
                        force_ema_alpha=bp.FORCE_EMA_ALPHA_LAPF,
                        inflation_margin_mm=bp.INFLATION_MARGIN_LAPF,
                        leash_length_mm=bp.LEASH_LENGTH_LAPF,
                        leash_half_angle_deg=bp.LEASH_HALF_ANGLE_DEG_LAPF,

                        lookahead=bp.LOOKAHEAD_LAPF,
                        advance_radius=bp.ADVANCE_RADIUS_LAPF,
                        blocking=False,
                        use_profile=self._use_profile,
                        accel=self._accel,
                        decel=self._decel
                    )

        elif self._stage == bp.NavStage.HEADING:
            if self._handle is None:
                print(f"Turning to: {self._goal_heading:.2f} deg")
                self._handle = self._robot.turn_to(
                    angle_deg=self._goal_heading,
                    blocking=False,
                    tolerance_deg=2.0,
                    timeout=None,
                    use_profile=self._use_profile,
                    accel=self._angular_accel,
                    decel=self._angular_decel
                )

    def is_done(self) -> bool:
        return self._is_done

    def cancel(self) -> None:
        if self._handle:
            self._handle.cancel()
            self._handle = None
        super().cancel()

    def on_exit(self) -> None:
        """Write waypoints and vertices to mission data."""
        for wp in self._waypoints: self._mission_data["waypoints"].append(wp)
        for v in self._vertices: self._mission_data["vertices"].append(v)


class WaitTask(Task):
    def __init__(self, robot: Robot, mission_data: dict, params: dict):
        super().__init__(robot, mission_data, params)
        self._is_done = False
        self._handle = None
        self._time_pause_period = 2.0
        self._time_pause_start = None
        self._time_detection_period = 2.0
        self._time_last_detection = 0.0
        self._traffic_light_detected = False

    def update(self):
        if self._is_done:
            return

        now = time.monotonic()

        if self._params.get("trigger") == "green_light":
            # 1. Check for traffic light first and print color
            if vh.sees_traffic_light(self._robot):
                self._traffic_light_detected = True
                self._robot.stop()
                if self._handle:
                    self._handle.cancel()
                    self._handle = None
                detected_color = vh.find_traffic_light_color(self._robot)
                if now - self._time_last_detection >= self._time_detection_period:
                    self._time_last_detection = now
                    if detected_color is not None:
                        print(f"[WaitTask] Traffic light detected, color {detected_color}")
                    else:
                        print("[WaitTask] Traffic light detected, color undefined")
                if detected_color == "green":
                    self._is_done = True
                    print("[WaitTask] Completion triggered by green light")
                return # Don't turn if we see a light (even if it's red)

            # 2. Handle the turn/pause search cycle
            if self._handle is not None:
                # We are currently in the middle of a turn
                if self._handle.is_done():
                    self._handle = None
                    self._time_pause_start = now
                    print("[WaitTask] Turn complete, pausing for vision...")
            elif not self._traffic_light_detected:
                # We are currently in a pause period
                if now - self._time_pause_start >= self._time_pause_period:
                    print("[WaitTask] Pause complete, rotating search...")
                    self._handle = self._robot.turn_by(
                        delta_deg=20.0,
                        blocking=False,
                        tolerance_deg=2.0,
                        timeout=None,
                        use_profile=self._params.get("use_profile", False),
                        accel=self._params.get("angular_accel", None),
                        decel=self._params.get("angular_decel", self._params.get("angular_accel"))
                    )

    def on_enter(self) -> None:
        self._time_pause_start = time.monotonic()
        print(f"[WaitTask] Searching for traffic light")

    def on_exit(self) -> None:
        self._robot.stop()
        print(f"[WaitTask] EXIT")

    def cancel(self) -> None:
        if self._handle:
            self._handle.cancel()
            self._handle = None
        super().cancel()

    def is_done(self) -> bool:
        return self._is_done


class ManipTask(Task):
    def __init__(self, robot: Robot, mission_data: dict, params: dict):
        super().__init__(robot, mission_data, params)
        self._is_done = False
        self._mission_data = mission_data
        self._init_pose = self._robot.get_pose()
        
        self._command = self._params.get("command")
        if self._command == "pick":
            ingredients = self._mission_data.get("ingredients")
            ingredient_idx = self._mission_data.get("ingredient_idx")
            self._ingredient = ingredients[ingredient_idx]

            self._sequence = bp.PICK_SEQUENCE
            self._level_height = hm.LIFT_LIFTOFF_STEPS - self._ingredient.HEIGHT_STEPS
            self._close_deg = self._ingredient.GRIP_ANGLE
            self._raise_steps = hm.LIFT_LIFTOFF_STEPS
            if ingredient_idx + 1 < len(ingredients):
                self._raise_steps -= ingredients[ingredient_idx + 1].HEIGHT_STEPS
            else:
                self._raise_steps -= hm.LIFT_LIFTOFF_BUFFER_STEPS

        elif self._command == "place":
            self._sequence = bp.PLACE_SEQUENCE
            self._level_height = hm.LIFT_LIFTOFF_STEPS - hm.LIFT_LIFTOFF_BUFFER_STEPS
            self._raise_steps = hm.LIFT_LIFTOFF_STEPS
            for ingredient in self._mission_data["burger_stack"]:
                self._raise_steps -= ingredient.HEIGHT_STEPS

        else:
            self._sequence = []
            self._is_done = True
     
        self._idx = 0
        self._handle = None
        self._stage_init = True

    def on_enter(self) -> None:
        """Enable manipulator hardware once at task start."""
        self._robot.step_enable(hm.LIFT_STEPPER_ID)
        self._robot.enable_servo(hm.GRIPPER_SERVO_ID)
        print(f"Use profile: {self._params.get("use_profile")}")

    def on_exit(self) -> None:
        """Disable power-hungry manipulator hardware upon exit."""
        self._robot.step_disable(hm.LIFT_STEPPER_ID)
        if self._command == "pick":
            self._mission_data["burger_stack"].append(self._ingredient)

    def update(self):
        if self._is_done:
            return

        stage = self._sequence[self._idx]

        if self._stage_init:
            print(f"[Maniptask] {stage.name}")
            if stage == bp.ManipStage.LEVEL:
                self._handle = fh.move_lift(
                    robot=self._robot,
                    position=self._level_height,
                    move_type=hm.StepMoveType.ABSOLUTE,
                    disable_on_done=False
                )
            elif stage == bp.ManipStage.OPEN:
                self._handle = fh.set_gripper(
                    robot=self._robot,
                    angle=hm.GRIPPER_OPEN_DEG
                )
            elif stage == bp.ManipStage.CLOSE:
                self._handle = fh.set_gripper(
                    robot=self._robot,
                    angle=self._close_deg
                )
            elif stage == bp.ManipStage.FORWARD:
                self._handle = fh.approach_ingredient_table(self._robot)
            elif stage == bp.ManipStage.RETREAT:
                x, y, _ = self._robot.get_pose()
                x_init, y_init, _ = self._init_pose
                backoff_distance = math.dist([x, y], [x_init, y_init]) + 45.0
                
                print(f"[ManipTask] Retreating {backoff_distance:.1f} mm, use profile {self._params.get("use_profile")}")
                self._handle = self._robot.move_backward(
                    distance=backoff_distance,
                    velocity=self._params.get("velocity", bp.APPROACH_VEL_MM_S),
                    tolerance=bp.TOLERANCE_MM,
                    blocking=False,
                    use_profile=self._params.get("use_profile", False),
                    accel=self._params.get("accel", None),
                    decel=self._params.get("decel", self._params.get("accel"))
                )
            elif stage == bp.ManipStage.LOWER:
                self._handle = fh.move_lift(
                    robot=self._robot,
                    position=hm.LIFT_LIFTOFF_STEPS,
                    move_type=hm.StepMoveType.ABSOLUTE,
                    disable_on_done=False
                )
            elif stage == bp.ManipStage.RAISE:
                self._handle = fh.move_lift(
                    robot=self._robot,
                    position=self._raise_steps,
                    move_type=hm.StepMoveType.ABSOLUTE,
                    disable_on_done=False
                )
            
            self._stage_init = False

        # Check for completion
        done = False
        if self._handle:
            if self._handle.is_done():
                self._handle = None
                done = True
        
        if done:
            self._idx += 1
            if self._idx < len(self._sequence):
                self._stage_init = True
            else:
                self._is_done = True
                self._mission_data["ingredient_idx"] += 1

    def cancel(self) -> None:
        if self._handle:
            self._handle.cancel()
            self._handle = None
        super().cancel()

    def is_done(self) -> bool:
        return self._is_done


class IdentTask(Task):
    def __init__(self, robot: Robot, mission_data: dict, params: dict):
        super().__init__(robot, mission_data, params)
        self._is_done = False
        self._last_attempt_time = 0
        self._attempt_period = 2.0 # Wait 2s between identification attempts

    def update(self):
        if self._is_done:
            return

        now = time.monotonic()
        if now - self._last_attempt_time < self._attempt_period:
            return
        
        self._last_attempt_time = now
        
        if vh.capture_and_crop_identify_person(robot=self._robot, save_path=vh.IDENTIFY_PERSON_PATH):
            score_1 = vh.image_match_score(vh.IDENTIFY_PERSON_PATH, vh.SUSPECT_1_PATH)
            score_2 = vh.image_match_score(vh.IDENTIFY_PERSON_PATH, vh.SUSPECT_2_PATH)
            print(f"[IdentTask] Match results: Suspect1={score_1}, Suspect2={score_2}")

            if score_1 < vh.MIN_IMAGE_MATCH_SCORE and score_2 < vh.MIN_IMAGE_MATCH_SCORE:
                print("[IdentTask] UPDATE: No confident image match found. Will retry...")
            elif score_1 > score_2:
                self._mission_data["matched_customer"] = 1
                self._mission_data["delivery_status"] = False
                self._is_done = True
                print("[IdentTask] UPDATE: Matched to customer 1.")
            elif score_2 > score_1:
                self._mission_data["matched_customer"] = 2
                self._mission_data["delivery_status"] = False
                self._is_done = True
                print("[IdentTask] UPDATE: Matched to customer 2.")
        else:
            print(f"[IdentTask] UPDATE: Capture and identification failure. Will retry...")

    def cancel(self) -> None:
        super().cancel()

    def is_done(self) -> bool:
        return self._is_done
    
    def on_enter(self):
        print("[IdentTask] ENTER: Analyzing target customer.")

    def on_exit(self):
        print("[IdentTask] EXIT: Customer match complete.")


class PauseTask(Task):
    def __init__(self, robot: Robot, mission_data: dict, params: dict):
        super().__init__(robot, mission_data, params)
        self._is_done = False
        self._time_pause = params.get("time_pause")
        self._time_pause_start = None

    def update(self):
        if self._is_done:
            return

        now = time.monotonic()
        if now - self._time_pause_start >= self._time_pause:
            self._is_done = True
            print("[WaitTask] Pause complete.")

    def on_enter(self) -> None:
        self._robot.stop()
        self._time_pause_start = time.monotonic()
        print(f"[PauseTask] Pausing for {self._time_pause} seconds.")

    def on_exit(self) -> None:
        print(f"[PauseTask] Resuming run.")

    def cancel(self) -> None:
        super().cancel()

    def is_done(self) -> bool:
        return self._is_done


def build_task(robot: Robot, task_dict: dict, mission_data: dict) -> Task:
    state = task_dict.get("state")

    if state == "NAV":
        return NavTask(robot, mission_data, task_dict)
    elif state == "WAIT":
        return WaitTask(robot, mission_data, task_dict)
    elif state == "MANIP":
        return ManipTask(robot, mission_data, task_dict)
    elif state == "IDENT":
        return IdentTask(robot, mission_data, task_dict)
    elif state == "HOME":
        return HomeTask(robot, mission_data, task_dict)
    elif state == "PAUSE":
        return PauseTask(robot, mission_data, task_dict)
    else:
        return Task(robot, mission_data, task_dict)
