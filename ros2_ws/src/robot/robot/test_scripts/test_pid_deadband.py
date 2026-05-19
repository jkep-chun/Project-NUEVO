from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    INITIAL_THETA_DEG,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    WHEEL_BASE,
    WHEEL_DIAMETER,
    Motor,
    DCPidLoop
)
from robot.robot import FirmwareState, Robot

DUR = 6.0

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


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)
    robot.set_pid_gains(Motor.DC_M1, DCPidLoop.VELOCITY, 0.17, 0.09, 0.0)
    robot.set_pid_gains(Motor.DC_M2, DCPidLoop.VELOCITY, 0.17, 0.09, 0.0)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    running_start_time = 0.0

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            robot.reset_odometry()
            if not robot.wait_for_odometry_reset(timeout=2.0):
                print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
                robot.wait_for_pose_update(timeout=0.5)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 to start")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                robot.reset_odometry()
                if not robot.wait_for_odometry_reset(timeout=2.0):
                    print("[warn] odometry reset not confirmed within 2.0s; continuing with latest pose")
                    robot.wait_for_pose_update(timeout=0.5)
                show_running_leds(robot)
                print(f"[FSM] RUNNING — commanding 120mm/s for {DUR:.1f}s")
                robot.set_velocity(120.0, 0.0)
                show_running_leds(robot)
                running_start_time = time.monotonic()
                state = "RUNNING"

        elif state == "RUNNING":
            if time.monotonic() - running_start_time > DUR:
                robot.stop()
                show_idle_leds(robot)
                print("[FSM] IDLE — press BTN_1 to start")
                state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
