from __future__ import annotations

import math
import time
import os
import numpy as np

from robot.fsm_helpers.path_helpers import generate_open_rounded_path

from robot.robot import Robot
import robot.hardware_map as hm
import robot.fsm_helpers.burgerbot_parameters as bp
import robot.fsm_helpers.course_parameters as cp
import robot.fsm_helpers.vision_helpers as vh
import robot.fsm_helpers.firmware_helpers as fh

ENABLE_CAM = True

class Task:
    def __init__(self, robot: Robot, mission_data: dict):
        self.robot = robot
        self.mission_data = mission_data

    def update(self) -> None:
        pass

    def is_done(self) -> bool:
        return True
    
    def on_enter(self) -> None:
        pass

    def on_exit(self) -> None:
        pass


class NavTask(Task):
    def __init__(self, robot, mission_data, waypoints, goal_heading=None, path_planner="pp", delivery_segment=False):
        super().__init__(robot, mission_data)
        # Ensure waypoints is a list of tuples
        if waypoints is not None:
            self.vertices = [tuple(wp) for wp in np.atleast_2d(waypoints)]
        else:
            self.vertices = []
        
        self.goal_heading = goal_heading
        self.path_planner = path_planner
        self.mission_data = mission_data
        self.delivery_segment = delivery_segment

        self._is_done = False
        self.wp_lapf_idx = 0
        self.handle = None
        self.start_heading = None
        self._start_traj_idx = len(self.robot._fused_traj)

        self.stage_idx = 0
        self.stage_sequence = []
        self.waypoints = list(self.vertices) # Fallback to original vertices

        if self.vertices:
            self.stage_sequence.append(bp.NavStage.POSITION)
            if self.path_planner == "pp":
                # Make a dense rounded path if in pure pursuit
                x, y, _ = self.robot.get_pose()
                path_vertices = [(x, y)] + self.vertices
                self.waypoints = generate_open_rounded_path(
                    vertices=path_vertices,
                    R=bp.TURN_RADIUS,
                    ds=bp.WP_SPACING
                )
                # Add an initial heading pivot if in purepursuit
                # Find first waypoint far enough to justify a pivot
                for wp in self.waypoints:
                    dx = wp[0] - x
                    dy = wp[1] - y
                    if math.hypot(dx, dy) > bp.TOLERANCE_MM:
                        self.start_heading = math.degrees(math.atan2(dy, dx))
                        self.stage_sequence.insert(0, bp.NavStage.START_HEADING)
                        break
        
        # Add goal heading if provided
        if self.goal_heading is not None:
            self.stage_sequence.append(bp.NavStage.HEADING)

        if self.stage_sequence:
            # Set stage sequence
            self.stage = self.stage_sequence[self.stage_idx]
        else:
            self.stage = None
            self._is_done = True

    def update(self):

        if self.handle and self.handle.is_done():
            self.handle = None
            if self.stage == bp.NavStage.POSITION and self.path_planner == "lapf" and self.wp_lapf_idx < len(self.waypoints) - 1:
                # Stay in bp.NavStage.POSITION
                self.wp_lapf_idx += 1
            else:
                self.stage_idx += 1
                if self.stage_idx < len(self.stage_sequence):
                    self.stage = self.stage_sequence[self.stage_idx]
                else:
                    if self.delivery_segment:
                        self.mission_data["delivery_status"] = True
                    self._is_done = True
                    return

        if self.stage == bp.NavStage.START_HEADING:
            if self.handle is None:
                print(f"Turning to face first waypoint: {self.start_heading:.2f} deg")
                self.handle = self.robot.turn_to(
                    angle_deg=self.start_heading,
                    blocking=False,
                    tolerance_deg=5.0,
                    timeout=None
                )

        elif self.stage == bp.NavStage.POSITION:
            if self.handle is None:
                if self.path_planner == "pp":
                    print(f"Driving seg with {len(self.waypoints)} waypoints")
                    self.handle = self.robot.purepursuit_follow_path(
                        waypoints=self.waypoints,
                        velocity=bp.VELOCITY_MM_S,
                        lookahead=bp.LOOKAHEAD_MM,
                        tolerance=bp.TOLERANCE_MM,
                        blocking=False,
                        max_angular_rad_s=1.0,
                        advance_radius=bp.ADVANCE_RADIUS_MM
                    )
                elif self.path_planner == "lapf":
                    wp = self.waypoints[self.wp_lapf_idx]
                    tx, ty = wp[0], wp[1]
                    print(f"Driving (LAPF) toward: ({tx:.2f},{ty:.2f})")
                    self.handle = self.robot.lapf_to_goal(
                        x=tx, y=ty,
                        velocity=bp.VELOCITY_MM_S*0.8,
                        tolerance=bp.TOLERANCE_MM_LAPF,
                        leash_length_mm=50,
                        repulsion_range_mm=150.0, # 350.0
                        target_speed_mm_s=bp.VELOCITY_MM_S,
                        repulsion_gain=150, # 550.0,
                        attraction_gain=1.0,
                        force_ema_alpha=0.35,
                        inflation_margin_mm=150.0,
                        leash_half_angle_deg=25.0,
                        blocking=False
                    )

        elif self.stage == bp.NavStage.HEADING:
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

        # Plot current task waypoints (densified path)
        if self.waypoints:
            wx = [wp[0] for wp in self.waypoints]
            wy = [wp[1] for wp in self.waypoints]
            plt.plot(wx, wy, color='green', linestyle='--', alpha=0.5, label='Waypoints')
            
            if len(wx) > 0:
                plt.text(wx[0], wy[0], ' Start', color='green', verticalalignment='bottom', fontsize=9)
                plt.text(wx[-1], wy[-1], ' End', color='red', verticalalignment='bottom', fontsize=9)

        # Plot current task vertices (original given points)
        if self.vertices:
            vx = [v[0] for v in self.vertices]
            vy = [v[1] for v in self.vertices]
            plt.scatter(vx, vy, color='purple', marker='o', s=65, label='Vertices', zorder=4)

        # Current robot pose
        curr_x, curr_y, _ = self.robot.get_pose()
        plt.scatter([curr_x], [curr_y], color='red', marker='X', s=100, label='Final Position', zorder=10)

        plt.xlabel('X (mm)')
        plt.ylabel('Y (mm)')
        plt.title(f'Robot Navigation Trajectory ({self.path_planner.upper()})')
        plt.legend()
        plt.axis('equal')
        
        # Configure grid lines at SQ intervals, offset so (0,0) is centered in a square
        from matplotlib.ticker import MultipleLocator
        ax = plt.gca()
        # Offset is SQ/2 because (0,0) is the center of the square
        offset = cp.SQ / 2.0
        ax.xaxis.set_major_locator(MultipleLocator(cp.SQ, offset=-offset))
        ax.yaxis.set_major_locator(MultipleLocator(cp.SQ, offset=-offset))
        
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
    def __init__(self, robot, mission_data, params: dict):
        super().__init__(robot, mission_data)
        self._params = params
        self._is_done = False
        self._handle = None
        self._time_pause_period = 2.0
        self._time_pause_start = None
        self._time_detection_period = 2.0
        self._time_last_detection = 0.0
        self._traffic_light_detected = False

    def update(self):
        now = time.monotonic()

        if self.robot.was_button_pressed(hm.Button.BTN_1):
            self._is_done = True
            print("[WaitTask] Completion triggered by BTN_1")
            return

        if self._params.get("trigger") == "green_light":
            # 1. Check for traffic light first (Priority)
            if vh.sees_traffic_light(self.robot):
                self._traffic_light_detected = True
                if now - self._time_last_detection >= self._time_detection_period:
                    self._time_last_detection = now
                    print("[WaitTask] Traffic light detected")
                self.robot.stop()
                if self._handle:
                    self._handle.cancel()
                    self._handle = None
                
                detected_color = vh.find_traffic_light_color(self.robot)
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
                    self._handle = self.robot.turn_by(
                        delta_deg=20.0,
                        blocking=False,
                        tolerance_deg=2.0,
                        timeout=None
                    )

    def on_enter(self) -> None:
        self._time_pause_start = time.monotonic()
        print(f"[WaitTask] Searching for traffic light")

    def on_exit(self) -> None:
        self.robot.stop()
        print(f"[WaitTask] EXIT")

    def is_done(self) -> bool:
        return self._is_done


class ManipTask(Task):
    def __init__(self, robot, mission_data, params: dict):
        super().__init__(robot, mission_data)
        self._is_done = False
        cmd = params.get("cmd")
        if cmd == "pick":
            self.sequence = bp.PICK_SEQUENCE
        elif cmd == "place":
            self.sequence = bp.PLACE_SEQUENCE
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
            print(f"bp.ManipStage: {stage.name}")
            if stage == bp.ManipStage.LEVEL:
                self.robot.step_enable(hm.LIFT_STEPPER)
                self.robot.step_move(hm.LIFT_STEPPER, hm.LIFT_EXTEND_STEPS, hm.StepMoveType.ABSOLUTE, blocking=False)
                self.handle = fh.StepHomingHandle(self.robot, hm.LIFT_STEPPER) # Reuse StepHomingHandle for non-blocking move
            elif stage == bp.ManipStage.OPEN:
                self.robot.enable_servo(hm.GRIPPER_SERVO)
                self.robot.set_servo(hm.GRIPPER_SERVO, hm.GRIPPER_OPEN_DEG)
                self.timer_start = time.monotonic()
            elif stage == bp.ManipStage.CLOSE:
                self.robot.enable_servo(hm.GRIPPER_SERVO)
                self.robot.set_servo(hm.GRIPPER_SERVO, hm.GRIPPER_CLOSE_DEG)
                self.timer_start = time.monotonic()
            elif stage == bp.ManipStage.FORWARD:
                self.handle = self.robot.move_forward(distance=100.0, velocity=bp.VELOCITY_MM_S, tolerance=bp.TOLERANCE_MM, blocking=False)
            elif stage == bp.ManipStage.RETREAT:
                self.handle = self.robot.move_backward(distance=100.0, velocity=bp.VELOCITY_MM_S, tolerance=bp.TOLERANCE_MM, blocking=False)
            elif stage == bp.ManipStage.LOWER:
                self.robot.step_enable(hm.LIFT_STEPPER)
                self.robot.step_move(hm.LIFT_STEPPER, hm.LIFT_LOWER_STEPS, hm.StepMoveType.RELATIVE, blocking=False)
                self.handle = fh.StepHomingHandle(self.robot, hm.LIFT_STEPPER)
            elif stage == bp.ManipStage.RAISE:
                self.robot.step_enable(hm.LIFT_STEPPER)
                self.robot.step_move(hm.LIFT_STEPPER, hm.LIFT_BUFFER_STEPS, hm.StepMoveType.RELATIVE, blocking=False)
                self.handle = fh.StepHomingHandle(self.robot, hm.LIFT_STEPPER)
            
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
    def __init__(self, robot, mission_data):
        super().__init__(robot, mission_data)
        self._is_done = False
        self._last_attempt_time = 0
        self._attempt_period = 2.0 # Wait 2s between identification attempts

    def update(self):
        if self.robot.was_button_pressed(hm.Button.BTN_1):
            self._is_done = True
            print("[IdentTask] UPDATE: Completion triggered by BTN_1")
            return
        
        now = time.monotonic()
        if now - self._last_attempt_time < self._attempt_period:
            return
        
        self._last_attempt_time = now
        
        if vh.capture_and_crop_identify_person(robot=self.robot, save_path=vh.IDENTIFY_PERSON_PATH):
            score_1 = vh.image_match_score(vh.IDENTIFY_PERSON_PATH, vh.SUSPECT_1_PATH)
            score_2 = vh.image_match_score(vh.IDENTIFY_PERSON_PATH, vh.SUSPECT_2_PATH)
            print(f"[IdentTask] Match results: Suspect1={score_1}, Suspect2={score_2}")

            if score_1 < vh.MIN_IMAGE_MATCH_SCORE and score_2 < vh.MIN_IMAGE_MATCH_SCORE:
                print("[IdentTask] UPDATE: No confident image match found. Will retry...")
            elif score_1 > score_2:
                self.mission_data["matched_customer"] = 1
                self.mission_data["delivery_status"] = False
                self._is_done = True
                print("[IdentTask] UPDATE: Matched to customer 1.")
            elif score_2 > score_1:
                self.mission_data["matched_customer"] = 2
                self.mission_data["delivery_status"] = False
                self._is_done = True
                print("[IdentTask] UPDATE: Matched to customer 2.")
        else:
            print(f"[IdentTask] UPDATE: Capture and identification failure. Will retry...")

    def is_done(self) -> bool:
        return self._is_done
    
    def on_enter(self):
        print("[IdentTask] ENTER: Analyzing target customer.")

    def on_exit(self):
        print("[IdentTask] EXIT: Customer match complete.")


def build_task(robot: Robot, task_dict: dict, mission_data: dict) -> Task:
    state = task_dict.get("state")

    if state == "NAV":
        waypoints = list(task_dict.get("waypoints", []))
        goal_heading = task_dict.get("goal_heading")
        delivery_segment = False

        # Robustness: Inject customer data BEFORE path generation
        if not mission_data.get("delivery_status"):
            matched_customer = mission_data.get("matched_customer")
            if matched_customer == 1:
                waypoints.append(cp.WP_CUSTOMER_1)
                goal_heading = 0
                delivery_segment = True
            elif matched_customer == 2:
                waypoints.append(cp.WP_CUSTOMER_2)
                goal_heading = 0
                delivery_segment = True
            
        return NavTask(
            robot=robot,
            mission_data=mission_data,
            waypoints=waypoints,
            goal_heading=goal_heading,
            path_planner=task_dict.get("path_planner", "pp"),
            delivery_segment=delivery_segment
        )
    elif state == "WAIT":
        return WaitTask(robot, mission_data, task_dict)
    elif state == "MANIP":
        return ManipTask(robot, mission_data, task_dict)
    elif state == "IDENT":
        return IdentTask(robot, mission_data)
    else:
        return Task(robot, mission_data)
