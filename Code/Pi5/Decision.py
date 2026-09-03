"""
decision.py

First pass at decision logic for STARDUST robot.

Rule implemented: progressive (proportional) steering based on obstacle SIZE.
A bigger detected pillar (in pixels) means it's closer, so the robot should
steer away from it more sharply. This avoids a sudden hard-snap reaction —
the correction grows smoothly as the obstacle gets bigger/closer, instead of
staying at zero until some cutoff and then jumping to full steering.

This file intentionally does ONLY this one rule for now. Wall avoidance,
speed control, and safety overrides are meant to be added on top of this
later, not folded in yet.
"""


class RobotController:
    def __init__(self, frame_width=320, trigger_area=1200, max_area=4500, max_steer=1.0):
        """
        frame_width: width of the camera frame in pixels (used to center steering)
        trigger_area: pillar area (px^2) below which we ignore it (too far away to matter)
        max_area: pillar area (px^2) at which steering should be at its max (very close)
        max_steer: the steering value to return at full intensity (e.g. 1.0 = max turn)
        """
        self.frame_center_x = frame_width // 2
        self.trigger_area = trigger_area
        self.max_area = max_area
        self.max_steer = max_steer

    def decide(self, detections):
        """
        detections: {"pillars": [...], ...}   (walls/line ignored for now)
        Returns: {"steering": -1.0 to 1.0, "speed": placeholder for now}
        """
        pillars = detections.get("pillars", [])
        target = self._find_closest_pillar(pillars)

        steering = 0.0
        if target is not None:
            steering = self._progressive_steer(target)

        return {"steering": steering, "speed": 1.0}  # speed control not implemented yet

    def _find_closest_pillar(self, pillars):
        """Closest = biggest area. Ignore anything under trigger_area (too far to react to)."""
        candidates = [p for p in pillars if p.get("area", 0) >= self.trigger_area]
        if not candidates:
            return None
        return max(candidates, key=lambda p: p["area"])

    def _progressive_steer(self, pillar):
        """
        Core rule: steering magnitude scales smoothly with how far the pillar's
        area is between trigger_area (just noticed) and max_area (very close).

        area <= trigger_area  -> 0% steering
        area >= max_area      -> 100% steering (max_steer)
        in between            -> linear ramp

        Direction: red -> steer left (positive), green -> steer right (negative).
        Unknown color -> no steering (fail safe: don't guess a direction).
        """
        area = pillar["area"]
        color = pillar.get("color")

        span = self.max_area - self.trigger_area
        if span <= 0:
            intensity = 1.0
        else:
            intensity = (area - self.trigger_area) / span
        intensity = max(0.0, min(1.0, intensity))  # clamp to 0.0-1.0

        magnitude = intensity * self.max_steer

        if color == "red":
            direction = -1.0
        elif color == "green":
            direction = 1.0
        else:
            return 0.0

        return direction * magnitude

    def _progressive_speed(self, pillar):
        """
        Slows down proportionally to how much bigger (closer) the pillar has
        gotten between trigger_area and max_area — same ramp used for steering,
        just applied to speed instead. E.g. if the pillar is 10% of the way from
        trigger_area to max_area, speed drops 10% from full toward min_speed.
 
        area <= trigger_area  -> full speed (1.0)
        area >= max_area      -> min_speed (slowest allowed)
        in between            -> linear ramp down
        """
        area = pillar["area"]
 
        span = self.max_area - self.trigger_area
        if span <= 0:
            intensity = 1.0
        else:
            intensity = (area - self.trigger_area) / span
        intensity = max(0.0, min(1.0, intensity))  # clamp to 0.0-1.0
 
        # intensity=0 -> speed=1.0 (full) ; intensity=1 -> speed=min_speed
        speed = 1.0 - intensity * (1.0 - self.min_speed)
        return speed


    