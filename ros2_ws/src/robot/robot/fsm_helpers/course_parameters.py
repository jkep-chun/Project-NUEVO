"""
Course-specific geometry
"""

SQ = 609.6 # Length of one square in mm

EXTRA = 25.0 # Extra distance traveled over ramp (mm) # TODO: Tune (HIGH)

STEP_LEVEL_POSITION = 1000 # TODO: Measure
MAX_TURN_FOR_TRAFFIC_LIGHT_DEG = 30 # TODO: Verify

POSE_TRAFFIC_LIGHT  = (0*SQ,    0*SQ)
POSE_BURGER_BUN_1   = (0*SQ,    1.75*SQ)
POSE_BURGER_PATTY   = (0*SQ,    2*SQ)
POSE_BURGER_BUN_2   = (0*SQ,    2.25*SQ)
POSE_FACE           = (3.5*SQ,  6*SQ - EXTRA)
POSE_CUSTOMER_1     = (4*SQ,    2.25*SQ - EXTRA)
POSE_CUSTOMER_2     = (4*SQ,    1.75*SQ - EXTRA)
POSE_STOP_SIGN      = (4*SQ,    1.5*SQ - EXTRA)

WP1 = (0*SQ,    6*SQ)
WP2 = (1*SQ,    6*SQ)
WP3 = (1*SQ,    1*SQ - EXTRA)
WP4 = (2.5*SQ,  1*SQ - EXTRA)
WP5 = (4*SQ,    6*SQ - EXTRA)
WP6 = (4*SQ,    0*SQ - EXTRA)