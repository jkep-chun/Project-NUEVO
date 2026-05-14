# ---------------------------------------------------------------------------
# Venue placement configuration (TODO: Replace w/ real values or automate)
# ---------------------------------------------------------------------------
TOLERANCE_MM = 50 # TODO: Tune
VELOCITY_MM_S = 120.0

POSE_TRAFFIC_LIGHT = (0, 0, 90)
POSE_BURGER_PATTY = (0, 1828.8, 180)
POSE_BURGER_BUN = (0, 1219.2, 180)
POSE_CUSTOMER_TARGET = (2438.4, 3048, -90)
POSE_CUSTOMER_A = (2438.4, 2438.4, 0)
POSE_STOP_SIGN = (2438.4, 609.6, -90)

# ----- Tasks ---------------------------------------------------------------

tasks = [
    {"state": "WAIT", "trigger": "green_light"},
    {"state": "NAV",  "visualTarget": "burger_bun", "goal_pose_mm": POSE_BURGER_BUN},
    # {"state": "MANIP", "cmd": "pick"},
    # {"state": "NAV",  "visualTarget": "burger_patty", "goal_pose_mm": POSE_BURGER_PATTY},
    # {"state": "MANIP", "cmd": "pick"},
    # {"state": "NAV",  "visualTarget": "burger_bun", "goal_pose_mm": POSE_BURGER_BUN},
    # {"state": "MANIP", "cmd": "pick"},
    # {"state": "NAV",  "visualTarget": "customer_shown", "goal_pose_mm": POSE_CUSTOMER_TARGET},
    # {"state": "IDENT"},
    # {"state": "NAV",  "visualTarget": "customer", "goal_pose_mm": POSE_CUSTOMER_A},
    # {"state": "MANIP", "cmd": "place"},
    # {"state": "NAV",  "visualTarget": "stop_sign", "goal_pose_mm": POSE_STOP_SIGN},
]