"""
User-defined mission tasks. Set MODE below
"""

from robot.fsm_helpers import course_parameters as cp

MODE = "TEST_NAV"

if MODE == "TEST_LAPF":
    tasks = [
        {"state": "NAV", "waypoints": (0, 3*610.0, 0)}
    ]

elif MODE == "TEST_CAM":
    tasks = [
        {"state": "WAIT", "trigger": "green_light"}
    ]

elif MODE == "TEST_NAV":
    tasks = [
        {"state": "NAV", "path_planner": "pp", "vision": "burger_bun", "waypoints": cp.POSE_BURGER_BUN_1},
        {"state": "NAV", "path_planner": "pp", "vision": "burger_patty", "waypoints": cp.POSE_BURGER_PATTY},
        {"state": "NAV", "path_planner": "pp", "vision": "burger_bun", "waypoints": cp.POSE_BURGER_BUN_2},
        {"state": "NAV", "path_planner": "pp", "waypoints": [cp.WP1, cp.WP2, cp.WP3, cp.WP4]},
        {"state": "NAV", "path_planner": "lapf", "vision": "face", "waypoints": cp.POSE_FACE},
        {"state": "NAV", "path_planner": "pp", "vision": "customer", "waypoints": cp.POSE_CUSTOMER_A},
        {"state": "NAV", "path_planner": "pp", "vision": "stop_sign", "waypoints": cp.POSE_STOP_SIGN},
        {"state": "NAV", "path_planner": "pp", "waypoints": cp.WP5}
    ]

elif MODE == "FULL_RUN":
    tasks = [
        {"state": "WAIT", "trigger": "green_light"},
        {"state": "NAV", "path_planner": "pp", "vision": "burger_bun", "waypoints": cp.POSE_BURGER_BUN_1},
        {"state": "MANIP", "cmd": "pick"},
        {"state": "NAV", "path_planner": "pp", "vision": "burger_patty", "waypoints": cp.POSE_BURGER_PATTY},
        {"state": "MANIP", "cmd": "pick"},
        {"state": "NAV", "path_planner": "pp", "vision": "burger_bun", "waypoints": cp.POSE_BURGER_BUN_2},
        {"state": "MANIP", "cmd": "pick"},
        {"state": "NAV", "path_planner": "pp", "waypoints": [cp.WP1, cp.WP2, cp.WP3, cp.WP4]},
        {"state": "NAV", "path_planner": "lapf", "vision": "face", "waypoints": cp.POSE_FACE},
        {"state": "IDENT"},
        {"state": "PLAN"},
        {"state": "NAV", "path_planner": "pp", "vision": "customer", "waypoints": cp.POSE_CUSTOMER_A},
        {"state": "MANIP", "cmd": "place"},
        {"state": "NAV", "path_planner": "pp", "vision": "stop_sign", "waypoints": cp.POSE_STOP_SIGN},
        {"state": "NAV", "path_planner": "pp", "waypoints": cp.WP5}
    ]