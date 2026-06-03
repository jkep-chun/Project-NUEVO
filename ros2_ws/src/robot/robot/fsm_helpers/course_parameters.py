"""
Course-specific geometry
"""

SQ = 609.6 # Length of one square in mm

EXTRA = 25.0 # Over ramp (mm) TODO: Tune (LOW)

MAX_TURN_FOR_TRAFFIC_LIGHT_DEG = 30 # TODO: Verify (HIGH)

WP_TRAFFIC_LIGHT  = (0*SQ,    0*SQ)
WP_BURGER_BUN_1   = (0*SQ,    1.75*SQ)
WP_BURGER_PATTY   = (0*SQ,    2*SQ)
WP_BURGER_BUN_2   = (0*SQ,    2.25*SQ)
WP_CUSTOMER_1     = (4*SQ,    2.25*SQ - EXTRA)
WP_CUSTOMER_2     = (4*SQ,    1.75*SQ - EXTRA)
WP_STOP_SIGN      = (4*SQ,    1.5*SQ - EXTRA)

WP1             = (0*SQ,    6*SQ)
WP2             = (1*SQ,    6*SQ)
WP3             = (1*SQ,    1*SQ - EXTRA)
WP4             = (2.5*SQ,  1*SQ - EXTRA)
WP_CUSTOMER_ID  = (3.25*SQ, 6*SQ - EXTRA)
WP5             = (4*SQ,    6*SQ - EXTRA)
WP6             = (4*SQ,    0*SQ - EXTRA)