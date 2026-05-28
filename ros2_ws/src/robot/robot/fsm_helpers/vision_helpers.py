from pathlib import Path
import cv2

from robot.robot import Robot

VISION_STALE_SEC = 0.5  # TODO: Tune
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.50
MIN_STOP_SIGN_CONFIDENCE = 0.50
MIN_PERSON_CONFIDENCE = 0.50

IMAGE_DIR = Path("runtime_output/vision")
IDENTIFY_PERSON_PATH = IMAGE_DIR / "identify_person.jpg"
SUSPECT_1_PATH = IMAGE_DIR / "suspect_1.jpg"
SUSPECT_2_PATH = IMAGE_DIR / "suspect_2.jpg"

CAMERA_INDEX = 10
MIN_IMAGE_MATCH_SCORE = 20

def find_traffic_light_color(robot: Robot) -> str | None:
    """Return the best recent red/green traffic-light result, or None."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None

    best_color = None
    best_confidence = -1.0

    for detection in robot.get_detections("traffic light"):
        confidence = float(detection["confidence"])
        if confidence < MIN_TRAFFIC_LIGHT_CONFIDENCE:
            continue

        attributes = detection.get("attributes", {})
        color_attribute = attributes.get("color", {})
        color = color_attribute.get("value")
        if color not in ("red", "green"):
            continue

        if confidence > best_confidence:
            best_confidence = confidence
            best_color = str(color)

    return best_color

def sees_stop_sign(robot: Robot) -> bool:
    """Return True if the robot sees a recent stop sign detection."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return False

    return robot.has_detection(
        "stop sign",
        min_confidence=MIN_STOP_SIGN_CONFIDENCE,
    )

def sees_person(robot: Robot) -> bool:
    """Return True if the robot sees a recent person detection."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return False

    return robot.has_detection(
        "person",
        min_confidence=MIN_PERSON_CONFIDENCE,
    )

def capture_photo(save_path: Path, camera_index: int = CAMERA_INDEX) -> bool:
    """Capture one frame from the camera and save it."""
    IMAGE_DIR.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        print("ERROR: Could not open camera.")
        return False

    ret, frame = cap.read()
    cap.release()

    if not ret or frame is None:
        print("ERROR: Could not read camera frame.")
        return False

    success = cv2.imwrite(str(save_path), frame)

    if not success:
        print(f"ERROR: Could not save image to {save_path}")
        return False

    print(f"Saved image to {save_path}")
    return True
    
def capture_identify_person() -> bool:
    """Capture and save the reference target image."""
    return capture_photo(IDENTIFY_PERSON_PATH)


def capture_suspect_1() -> bool:
    """Capture and save the first candidate target image."""
    return capture_photo(SUSPECT_1_PATH)


def capture_suspect_2() -> bool:
    """Capture and save the second candidate target image."""
    return capture_photo(SUSPECT_2_PATH)

def image_match_score(reference_path: Path, candidate_path: Path) -> int:
    """Compare two saved images using ORB feature matching. Higher score means more similar."""
    reference = cv2.imread(str(reference_path))
    candidate = cv2.imread(str(candidate_path))

    if reference is None:
        print(f"ERROR: Could not load reference image: {reference_path}")
        return 0

    if candidate is None:
        print(f"ERROR: Could not load candidate image: {candidate_path}")
        return 0

    reference_gray = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY)
    candidate_gray = cv2.cvtColor(candidate, cv2.COLOR_BGR2GRAY)

    orb = cv2.ORB_create(nfeatures=1000)

    kp1, des1 = orb.detectAndCompute(reference_gray, None)
    kp2, des2 = orb.detectAndCompute(candidate_gray, None)

    if des1 is None or des2 is None:
        print("WARNING: Not enough features found in one of the images.")
        return 0

    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = matcher.match(des1, des2)

    good_matches = [match for match in matches if match.distance < 60]

    return len(good_matches)

def choose_matching_suspect(min_score: int = MIN_IMAGE_MATCH_SCORE) -> str | None:
    """Return which suspect image best matches identify_person.

    Returns:
        "suspect_1" if suspect_1 matches better
        "suspect_2" if suspect_2 matches better
        None if neither match is strong enough
    """
    score_1 = image_match_score(IDENTIFY_PERSON_PATH, SUSPECT_1_PATH)
    score_2 = image_match_score(IDENTIFY_PERSON_PATH, SUSPECT_2_PATH)

    print(f"suspect_1 match score: {score_1}")
    print(f"suspect_2 match score: {score_2}")

    if score_1 < min_score and score_2 < min_score:
        print("No confident image match found.")
        return None

    if score_1 > score_2:
        return "suspect_1"

    if score_2 > score_1:
        return "suspect_2"

    print("Tie between suspect_1 and suspect_2.")
    return None
    # functions that:
        # gender detected, return male or female
    