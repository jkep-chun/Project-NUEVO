"""
User-defined mission tasks. Set MODE below
"""

from robot.fsm_helpers import course_parameters as cp
from robot.fsm_helpers.course_parameters import SQ

MODE = "ALL_WAYPOINTS"

# =============================================================================
# Navigation
# =============================================================================

if MODE == "TUNE_LAPF":
    tasks = [{"state": "NAV", "waypoints": (0, 3*610.0, 0)}]

elif MODE == "TUNE_TURN":
    tasks = [{
        "state": "NAV",
        "path_planner": "pp",
        #"waypoints": [(0*SQ, 6*SQ)],
        #"waypoints": [(0*SQ, 1*SQ), (1*SQ, 1*SQ)],
        "goal_heading": 0.0
    },
    {
        "state": "NAV",
        "path_planner": "pp",
        "goal_heading": -90.0
    },
    {
        "state": "NAV",
        "path_planner": "pp",
        "goal_heading": -180.0
    },
    {
        "state": "NAV",
        "path_planner": "pp",
        "goal_heading": 90.0
    },
    {
        "state": "NAV",
        "path_planner": "pp",
        "goal_heading": 0.0
    },
        {
        "state": "NAV",
        "path_planner": "pp",
        "goal_heading": -90.0
    },
    {
        "state": "NAV",
        "path_planner": "pp",
        "goal_heading": -180.0
    },
    {
        "state": "NAV",
        "path_planner": "pp",
        "goal_heading": 90.0
    }]

elif MODE == "SQUARE2":
    tasks = [
        {"state": "NAV", "path_planner": "pp", "waypoints": [
            (0*SQ, 0*SQ),
            (0*SQ, 2*SQ),
            (2*SQ, 2*SQ),
            (2*SQ, 0*SQ),
            (0*SQ, 0*SQ),
            (0*SQ, 2*SQ),
            (2*SQ, 2*SQ),
            (2*SQ, 0*SQ),
            (0*SQ, 0*SQ)
            ]
        }
    ]

elif MODE == "ALL_WAYPOINTS":
    tasks = [
        {"state": "NAV", "path_planner": "pp", "waypoints": [cp.WP1, cp.WP2, cp.WP3, cp.WP4]},
        {"state": "NAV", "path_planner": "lapf", "waypoints": [cp.POSE_FACE]},
        {"state": "NAV", "path_planner": "pp", "waypoints": [cp.WP5, cp.WP6]}
    ]

# =============================================================================
# Other single tasks
# =============================================================================

elif MODE == "IDENT":
    tasks = [{"state": "IDENT"}]

elif MODE == "WAIT":
    tasks = [{"state": "WAIT", "trigger": "green_light"}]

# =============================================================================
# Multiple tasks
# =============================================================================

elif MODE == "TEST_NAV":
    tasks = [
        {"state": "NAV", "path_planner": "pp", "vision": "burger_bun", "waypoints": cp.POSE_BURGER_BUN_1},
        {"state": "NAV", "path_planner": "pp", "vision": "burger_patty", "waypoints": cp.POSE_BURGER_PATTY},
        {"state": "NAV", "path_planner": "pp", "vision": "burger_bun", "waypoints": cp.POSE_BURGER_BUN_2},
        {"state": "NAV", "path_planner": "pp", "waypoints": [cp.WP1, cp.WP2, cp.WP3, cp.WP4]},
        {"state": "NAV", "path_planner": "lapf", "vision": "face", "waypoints": cp.POSE_FACE},
        {"state": "NAV", "path_planner": "pp", "vision": "customer", "waypoints": [cp.WP5, cp.POSE_CUSTOMER_A]},
        {"state": "NAV", "path_planner": "pp", "vision": "stop_sign", "waypoints": cp.POSE_STOP_SIGN},
        {"state": "NAV", "path_planner": "pp", "waypoints": cp.WP6}
    ]

elif MODE == "FINAL":
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