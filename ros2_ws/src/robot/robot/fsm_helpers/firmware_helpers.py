from robot.hardware_map import (
    Motor,
    DCPidLoop,
    LED,

    INITIAL_THETA_DEG,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    WHEEL_BASE,
    WHEEL_DIAMETER,

    LIFT_STEPPER,
    LIFT_HOME_VELOCITY,
    GRIPPER_SERVO,
    GRIPPER_LIMIT,
    GRIPPER_CLOSE_DEG,

    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,

    TAG_ID,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,

)
from robot.robot import FirmwareState, Robot

ENABLE_LIDAR = True
ENABLE_GPS = False

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )

    if ENABLE_LIDAR:
        robot.enable_lidar()
        robot.set_lidar_mount(
            x_mm=LIDAR_MOUNT_X_MM,
            y_mm=LIDAR_MOUNT_Y_MM,
            theta_deg=LIDAR_MOUNT_THETA_DEG,
        )
        robot.set_lidar_filter(
            range_min_mm=LIDAR_RANGE_MIN_MM,
            range_max_mm=LIDAR_RANGE_MAX_MM,
            fov_deg=LIDAR_FOV_DEG,
        )
        robot.start_lidar_world_publisher()
        print("[sensor] lidar enabled — subscribing to /scan")

    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        robot.set_tag_body_offset(TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM)
        print(f"[sensor] GPS enabled — tracking ArUco tag {TAG_ID}")


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.set_pid_gains(Motor.DC_M1, DCPidLoop.VELOCITY, 0.17, 0.09, 0.0)
    robot.set_pid_gains(Motor.DC_M2, DCPidLoop.VELOCITY, 0.17, 0.09, 0.0)


def reset_mission_pose(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
        robot.wait_for_pose_update(timeout=0.5)
        x, y, theta = robot.get_pose()
        print(f"POSE: ({x:.1f}, {y:.1f}, {theta:.1f})")


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


def cancel_motion(robot: Robot, handle) -> None:
    if handle is not None:
        handle.cancel()
        handle.wait(timeout=1.0)
    robot.stop()

class StepHomingHandle:
    """
    Handle to track non-blocking homing progress.
    Mimics the interface of MotionHandle.
    """
    def __init__(self, robot: Robot, stepper_id: int):
        self._robot = robot
        self._id = stepper_id
        self._saw_active = False

    def is_done(self) -> bool:
        """Returns True once homing starts and then returns to IDLE."""
        state = self._robot.get_step_state()
        if state is None:
            return False
        
        # Index is 0-based
        motion_state = state.steppers[self._id - 1].motion_state
        
        # 1. Detect that it has actually started moving
        if motion_state != 0: # 0 is IDLE
            self._saw_active = True
            
        # 2. Return True only if it was moving and is now IDLE
        if self._saw_active and motion_state == 0:
            self._robot.step_disable(self._id) # Safe to disable now
            return True
        return False    
   
class ServoHomingHandle:
    """
    Handle to track non-blocking homing progress.
    Mimics the interface of MotionHandle.
    """
    def __init__(self, robot: Robot, servo_id: int, limit_switch_id: int):
        self._robot = robot
        self._servo_id = servo_id
        self._limit_switch_id = limit_switch_id

        # Try to initialize current angle from existing state
        state = self._robot.get_servo_state()
        if state:
            pulse_us = state.channels[self._servo_id - 1].pulse_us
            self._current_angle = (pulse_us - 1000.0) * 180.0 / 1000.0
        else:
            self._current_angle = GRIPPER_CLOSE_DEG

        # Command initial position to start the sequence
        self._robot.set_servo(self._servo_id, self._current_angle)

    def is_done(self) -> bool:
        """Returns True once limit switch is triggered or 0 deg reached."""
        if self._robot.get_limit(self._limit_switch_id):
            return True

        if self._current_angle <= 0:
            self._robot.set_servo(self._servo_id, 0.0)
            return True

        # Move one step towards 0
        self._current_angle -= 1.0 
        self._robot.set_servo(self._servo_id, self._current_angle)
        return False


def home_lift(robot: Robot) -> StepHomingHandle:
    """
    Starts the non-blocking homing sequence for the lift.
    Returns a StepHomingHandle to poll for completion.
    """
    print("[HOMING] — Starting lift homing...")
    robot.step_enable(LIFT_STEPPER)
    robot.step_home(
        LIFT_STEPPER,
        direction=1,
        home_velocity=LIFT_HOME_VELOCITY,
        backoff_steps=50,
        blocking=False
    )
    return StepHomingHandle(robot, LIFT_STEPPER)

def home_gripper(robot: Robot) -> ServoHomingHandle:
    """
    Starts the non-blocking homing sequence for the grip.
    Returns a ServoHomingHandle to poll for completion.
    """
    print("[HOMING] — Starting gripper homing...")
    robot.enable_servo(GRIPPER_SERVO)
    return ServoHomingHandle(robot, GRIPPER_SERVO, GRIPPER_LIMIT)