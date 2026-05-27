from enum import IntEnum, auto

TOLERANCE_MM = 50.0
VELOCITY_MM_S = 150.0
LOOKAHEAD_MM = 304.8 # TODO: Tune
SPACING_MM = 50.0

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