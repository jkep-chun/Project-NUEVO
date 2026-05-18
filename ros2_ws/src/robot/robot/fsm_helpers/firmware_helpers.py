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
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("ODOMETRY RESET TIMEOUT")
    else:
        print("ODOMETRY RESET SUCCESS")
    robot.wait_for_pose_update(timeout=0.2)


class HomingHandle:
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
   
   
def home_lift(robot: Robot) -> HomingHandle:
    """
    Starts the non-blocking homing sequence for the lift.
    Returns a HomingHandle to poll for completion.
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
    return HomingHandle(robot, LIFT_STEPPER)