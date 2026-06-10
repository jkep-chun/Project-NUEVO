"""
User-defined mission tasks. Set MODE below
"""

from robot.fsm_helpers import course_parameters as cp
from robot.fsm_helpers import ingredient_helpers as ih
import robot.hardware_map as hm
from robot.fsm_helpers.course_parameters import SQ

ingredients = [ih.Bun, ih.Patty, ih.Bun]
tasks = [
    # /
    {
        "state": "NAV", 
        "path_planner": "pp", 
        "waypoints": [cp.WP1, cp.WP2, cp.WP3, cp.WP3B, cp.WP3C],
        "velocity": 310.0,
        "use_profile": True,
        "enable_slam_localization": True,
        "slam_position_fusion_alpha": 0.2,
        "slam_orientation_fusion_alpha": 0.2,
    },
    {
        "state": "NAV",
        "path_planner": "lapf",
        "waypoints": [cp.WP4B, cp.WP_CUSTOMER_ID],
        "velocity": 300.0,
        "goal_heading": 0.0,
        "enable_slam_localization": True,
    },
    {
        "state": "IDENT"
    },
    {
        "state": "NAV", 
        "path_planner": "pp",
        "waypoints": [cp.WP5],
        "velocity": 320.0,
        "use_profile": True,
        "goal_heading": 0.0,
        "enable_slam_localization": True,
        "slam_position_fusion_alpha": 0.08
    },
    {
        "state": "MANIP", "command": "place",
        "use_profile": True
    },
    {
        "state": "NAV", 
        "path_planner": "pp",
        "waypoints": [cp.WP5B],
        "velocity": 300.0,
        "use_profile": True,
        "enable_slam_localization": True,
        "slam_position_fusion_alpha": 0.08,
    },
    {
        "state": "PAUSE", 
        "time_pause": 2.5
    },
    {
        "state": "NAV", 
        "path_planner": "pp",
        "waypoints": [cp.WP6],
        "velocity": 300.0,
        "use_profile": True,
    },
]