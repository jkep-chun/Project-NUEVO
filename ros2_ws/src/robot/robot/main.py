"""
main.py — student entry point
==============================
This is the only file students are expected to edit.

The structure is intentionally simple:
- keep one plain `state` variable
- write helper functions for robot actions
- use `if state == "..."` inside the main loop

To run:
    ros2 run robot robot
"""

from __future__ import annotations
import time

from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.util import densify_polyline
from robot.path_planner import PurePursuitPlanner
import math


# ---------------------------------------------------------------------------
# Robot build configuration
# ---------------------------------------------------------------------------

POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 337.8
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True


def configure_robot(robot: Robot) -> None:
    """Apply the user unit plus robot-specific wheel mapping and odometry settings."""
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


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.ORANGE, 255)
    robot.set_led(LED.RED, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 255)
    robot.set_led(LED.RED, 0)

def show_error_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.RED, 255)


def start_robot(robot: Robot) -> None:
    """Start the firmware and reset odometry before the main mission begins."""
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)


def run(robot: Robot) -> None:
    configure_robot(robot)
    

    state = "INIT"
    drive_handle = None
    # FSM refresh rate control
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            # ENTRY
            start_robot(robot)
            print("[FSM] INIT (odometry reset)")

            # EXIT
            state = "IDLE"

        elif state == "IDLE":
            # ENTRY
            show_idle_leds(robot)
            print("[FSM] IDLE")
            # EXIT
            

        elif state == "NAV":
            # ENTRY
            show_running_leds(robot)
            print("[FSM] NAV")
            # LOOP
            # EXIT

        elif state == "WAIT":
            show_running_leds(robot)
            print("[FSM] WAIT (waiting for green light)")

        elif state == "ERR":
            show_error_leds(robot)
            print("[FSM] ERR")
            
        # FSM refresh rate control
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
