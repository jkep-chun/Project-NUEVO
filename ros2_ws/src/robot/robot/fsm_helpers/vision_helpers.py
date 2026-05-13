from robot.robot import Robot

LIGHT_HOLD_SEC = 2.0
VISION_STALE_SEC = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.50


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