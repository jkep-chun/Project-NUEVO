"""
User-defined mission tasks. Set MODE below
"""

from robot.fsm_helpers import course_parameters as cp
from robot.fsm_helpers import ingredient_helpers as ih
import robot.hardware_map as hm
from robot.fsm_helpers.course_parameters import SQ

tasks = [
    # {
    #     "state": "HOME",
    #     "lift_init_height": -ih.Bun.HEIGHT_STEPS + hm.LIFT_LIFTOFF_STEPS
    # },
    # {
    #     "state": "WAIT",
    #     "trigger": "green_light"
    # },
    # {
    #     "state": "NAV",
    #     "path_planner": "pp",
    #     "waypoints": cp.WP_BURGER_BUN_1, 
    #     "goal_heading": 180.0,
    #     "enable_slam_localization": True
    # },
    # {
    #     "state": "MANIP",
    #     "command": "pick",
    #     "ingredient": ih.Bun
    # },
    # {
    #     "state": "NAV",
    #     "path_planner": "pp",
    #     "waypoints": cp.WP_BURGER_PATTY,
    #     "goal_heading": 180.0,
    #     "enable_slam_localization": True
    # },
    # {
    #     "state": "MANIP",
    #     "command": "pick",
    #     "ingredient": ih.Patty
    # },
    # {
    #     "state": "NAV",
    #     "path_planner": "pp",
    #     "waypoints": cp.WP_BURGER_BUN_2,
    #     "goal_heading": 180.0,
    #     "enable_slam_localization": True
    # },
    # {
    #     "state": "MANIP", 
    #     "command": "pick", 
    #     "ingredient": ih.Bun
    # },
    {
        "state": "NAV", 
        "path_planner": "pp", 
        "waypoints": [cp.WP1, cp.WP2, cp.WP3, cp.WP3B, cp.WP3C],
        "enable_slam_localization": True
    },
    {
        "state": "NAV",
        "path_planner": "lapf",
        "waypoints": [cp.WP4B, cp.WP_CUSTOMER_ID],
        "goal_heading": 0.0,
        "enable_slam_localization": True
    },
    # {
    #     "state": "IDENT"
    # },
    # {
    #     "state": "NAV", 
    #     "path_planner": "pp",
    #     "waypoints": [cp.WP5],
    #     "goal_heading": 0.0,
    #     "enable_slam_localization": True
    # },
    # {
    #     "state": "MANIP", "command": "place"
    # },
    # {
    #     "state": "NAV", 
    #     "path_planner": "pp",
    #     "waypoints": [cp.WP5B],
    #     "enable_slam_localization": True
    # },
    # {
    #     "state": "PAUSE", 
    #     "time_pause": 2.5
    # },
    # {
    #     "state": "NAV", 
    #     "path_planner": "pp",
    #     "waypoints": [cp.WP6],
    # },
]