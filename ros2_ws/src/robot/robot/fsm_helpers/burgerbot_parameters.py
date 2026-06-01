from enum import IntEnum, auto

TOLERANCE_MM = 20.0
TOLERANCE_MM_LAPF = 100.0
VELOCITY_MM_S = 150.0
LOOKAHEAD_MM = 40.0
ADVANCE_RADIUS_MM = 40.0

class NavStage(IntEnum):
    START_HEADING = auto()
    POSITION = auto()
    HEADING = auto()

class ManipStage(IntEnum):
    LEVEL = auto()
    OPEN = auto()
    CLOSE = auto()
    FORWARD = auto()
    RETREAT = auto()
    RAISE = auto()
    LOWER = auto()

PICK_SEQUENCE = [
    ManipStage.LEVEL,
    ManipStage.FORWARD,
    ManipStage.OPEN,
    ManipStage.LOWER,
    ManipStage.CLOSE,
    ManipStage.RAISE,
    ManipStage.RETREAT
]

PLACE_SEQUENCE = [
    ManipStage.LEVEL,
    ManipStage.FORWARD,
    ManipStage.LOWER,
    ManipStage.OPEN,
    ManipStage.RAISE,
    ManipStage.RETREAT
]