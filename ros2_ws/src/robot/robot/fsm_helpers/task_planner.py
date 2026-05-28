"""
User-defined mission tasks. Set MODE below
"""

from robot.fsm_helpers import course_parameters

MODE = "TEST_IDENT"

if MODE == "TEST_LAPF":
    tasks = [
        {"state": "NAV", "goal_pose_mm": (0, 3*610.0, 0)}
    ]

elif MODE == "TEST_CAM":
    tasks = [
        {"state": "WAIT", "trigger": "green_light"}
    ]

elif MODE == "TEST_IDENT":
    tasks = [
        {"state": "IDENT"}
    ]
    
elif MODE == "FULL_RUN":
    tasks = [
        {"state": "WAIT", "trigger": "green_light"},
        {"state": "NAV",  "visualTarget": "burger_bun", "goal_pose_mm": course_parameters.POSE_BURGER_BUN},
        {"state": "MANIP", "cmd": "pick"},
        {"state": "NAV",  "visualTarget": "burger_patty", "goal_pose_mm": course_parameters.POSE_BURGER_PATTY},
        {"state": "MANIP", "cmd": "pick"},
        {"state": "NAV",  "visualTarget": "burger_bun", "goal_pose_mm": course_parameters.POSE_BURGER_BUN},
        {"state": "MANIP", "cmd": "pick"},
        {"state": "NAV",  "visualTarget": "customer_shown", "goal_pose_mm": course_parameters.POSE_CUSTOMER_TARGET},
        {"state": "IDENT"},
        {"state": "NAV",  "visualTarget": "customer", "goal_pose_mm": course_parameters.POSE_CUSTOMER_A},
        {"state": "MANIP", "cmd": "place"},
        {"state": "NAV",  "visualTarget": "stop_sign", "goal_pose_mm": course_parameters.POSE_STOP_SIGN},
        {"state": "NAV",  "goal_pose_mm": course_parameters.POSE_END_ZONE},
    ]