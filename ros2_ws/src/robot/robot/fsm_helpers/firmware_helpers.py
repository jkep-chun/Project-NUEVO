from robot.hardware_map import (
    INITIAL_THETA_DEG,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    WHEEL_BASE,
    WHEEL_DIAMETER,
    TAG_ID,
    LIFT_STEPPER,
    LIFT_HOME_VELOCITY,
    GRIPPER_SERVO,
    GRIPPER_CLOSE_DEG,
    GRIPPER_OPEN_DEG
)
from robot.robot import FirmwareState, Robot

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
    robot.set_tracked_tag_id(TAG_ID)


def start_robot(robot: Robot) -> None:
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)


def home_lift(robot: Robot) -> bool:
    print("[HOMING] — press BTN_3 to trigger the shared LIM1 input for stepper 1")
    robot.step_enable(LIFT_STEPPER)
    ok = robot.step_home(
        LIFT_STEPPER,
        direction=1,
        home_velocity=LIFT_HOME_VELOCITY,
        backoff_steps=50,
        blocking=True,
        timeout=15.0,
    )
    if not ok:
        print("[warn] arm homing timed out — check LIM1 or use BTN_3 to simulate it")
        robot.step_disable(LIFT_STEPPER)
        return False
    robot.step_disable(LIFT_STEPPER)
    return True