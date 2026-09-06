"""
Decision.py

Steering/speed decision logic for the robot.

Two sources of steering are computed every frame:
  1) obstacle avoidance angle, from the closest red/green pillar (if any)
  2) wall-following angle, a PD controller keeping a set gap from whichever
     wall we're following (left wall when direction == "left", right wall
     when direction == "right")

The obstacle angle always wins when a pillar is close enough to react to 
otherwise we fall back to the wall-following angle -- that fallback is also what carries the robot through corners
(the wall-follow error grows sharply as the wall recedes into a corner,
and swings to a hard turn once the wall disappears entirely).

Steering is normalized to [-1.0, 1.0]; which sign means "left" vs "right"
is an arbitrary convention
"""

import time

FRAME_W = 320
FRAME_H = 240


class RobotController:
    def __init__(self, frame_width=FRAME_W, frame_height=FRAME_H, trigger_area=1200, max_area=4500, max_steer=1.0, min_speed=0.35, wall_kp=0.010, wall_kd=0.05, follow_gap_px=70, wall_gone_grace_s=0.35, corner_anticipation_gain=0.6):
        """
        frame_width / frame_height: camera frame size in pixels
        trigger_area / max_area: pillar area (px^2) span over which obstacle
            steering ramps from 0% to 100% (see _obstacle_steer)
        max_steer: steering magnitude at full intensity (servo factor units)
        min_speed: speed factor used while cornering hard or dodging closely
        wall_kp / wall_kd: PD gains on the lateral gap error
        follow_gap_px: desired pixel gap to the followed wall's inner edge
        wall_gone_grace_s: how long the followed wall can be absent from a
            frame before we call it a corner and start a hard turn
        corner_anticipation_gain: how strongly to nudge steering toward the
            turn as the wall's bottom edge creeps toward the frame bottom,
            ahead of it disappearing outright
        """
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.trigger_area = trigger_area
        self.max_area = max_area
        self.max_steer = max_steer
        self.min_speed = min_speed

        self.wall_kp = wall_kp
        self.wall_kd = wall_kd
        self.follow_gap_px = follow_gap_px
        self.wall_gone_grace_s = wall_gone_grace_s
        self.corner_anticipation_gain = corner_anticipation_gain

        self.direction = "left"  # which wall we follow / which way we lap the mat

        self._prev_gap_error = 0.0
        self._prev_wall_time = None
        self._wall_missing_since = None

    def set_direction(self, direction):
        assert direction in ("left", "right")
        self.direction = direction

    # ---------- obstacles
    def _find_closest_pillar(self, pillars):
        # Closest = biggest area. Ignore anything under trigger_area (too far to react to)
        candidates = [p for p in pillars if p.get("area", 0) >= self.trigger_area]
        if not candidates:
            return None
        return max(candidates, key=lambda p: p["area"])

    def _obstacle_steer(self, pillars):
        """
        Steering to avoid the nearest pillar. Red
        pillars are passed on the robot's right, green pillars on the
        robot's left. Magnitude ramps smoothly with how close the pillar
        is (bigger on-screen area = closer), instead of snapping straight
        to full steering past some cutoff.
        """
        target = self._find_closest_pillar(pillars)
        if target is None:
            return None

        area = target["area"]
        color = target.get("color")

        span = self.max_area - self.trigger_area
        intensity = 1.0 if span <= 0 else (area - self.trigger_area) / span
        intensity = max(0.0, min(1.0, intensity))
        magnitude = intensity * self.max_steer

        if color == "red":
            obstacle_dir = 1.0    # pass on the robot's right
        elif color == "green":
            obstacle_dir = -1.0   # pass on the robot's left
        else:
            return None

        return obstacle_dir * magnitude

    # ---------- walls
    def _select_wall(self, walls):
        # Among the walls detected this frame, pick the nearest blob on the side we follow
        half = self.frame_width / 2.0
        if self.direction == "left":
            side_walls = [w for w in walls if w["center_x"] < half]
        else:
            side_walls = [w for w in walls if w["center_x"] >= half]
        if not side_walls:
            return None
        # "nearest" = the blob whose bottom edge sits lowest in the frame (closest to the robot)
        return max(side_walls, key=lambda w: (w["y"] + w["h"], w["area"]))

    def _gap_error(self, wall):
        """
        Signed distance (px) between the followed wall's inner edge and the
        desired gap. Positive means "steer right", negative means "steer
        left"
        """
        if self.direction == "left":
            inner_edge_x = wall["x"] + wall["w"]
            return inner_edge_x - self.follow_gap_px
        else:
            inner_edge_x = wall["x"]
            return (self.frame_width - self.follow_gap_px) - inner_edge_x

    def _wall_steer(self, walls):
        """
        Wall follower with two additive contributions:
          1) a PD term on the lateral gap error (_gap_error), which does
          the actual centering in the lane
          2) an anticipation term that grows as the wall's bottom edge
             creeps toward the bottom of the frame -- that's the wall
             closing in ahead of a corner -- and nudges the steering
             toward the turn before the wall disappears outright
        If the wall disappears for longer than wall_gone_grace_s, that's
        a corner in progress: steer hard into the turn until it reappears.

        Returns (steer, is_corner).
        """
        now = time.time()
        wall = self._select_wall(walls)
        turn_into_corner = -1.0 if self.direction == "left" else 1.0

        if wall is None:
            if self._wall_missing_since is None:
                self._wall_missing_since = now
            if now - self._wall_missing_since >= self.wall_gone_grace_s:
                return turn_into_corner * self.max_steer, True
            # Not gone long enough to call it a corner yet -- coast on the last error
            steer = self.wall_kp * self._prev_gap_error
            return max(-self.max_steer, min(self.max_steer, steer)), False

        self._wall_missing_since = None

        gap_error = self._gap_error(wall)

        dt = max(now - self._prev_wall_time, 1e-3) if self._prev_wall_time is not None else 0.0
        derivative = (gap_error - self._prev_gap_error) / dt if dt > 0 else 0.0

        self._prev_gap_error = gap_error
        self._prev_wall_time = now

        closing_in_px = max(0.0, (wall["y"] + wall["h"]) - self.frame_height * 0.6)
        anticipation = turn_into_corner * (closing_in_px / self.frame_height) * self.corner_anticipation_gain

        steer = self.wall_kp * gap_error + self.wall_kd * derivative + anticipation
        steer = max(-self.max_steer, min(self.max_steer, steer))
        return steer, False

    # ----------combined
    def decide(self, detections):
        """
        detections: {"pillars": [...], "walls": [...], ...}
        Returns {"steering": -1.0..1.0, "speed": 0.0..1.0, "is_corner": bool}

        Priority: the obstacle-avoidance angle wins whenever a pillar is
        close enough to react to; otherwise fall back to the wall-following
        angle.
        """
        pillars = detections.get("pillars", [])
        walls = detections.get("walls", [])

        obstacle_steer = self._obstacle_steer(pillars)
        wall_steer, is_corner = self._wall_steer(walls)

        if obstacle_steer is not None:
            steering = obstacle_steer
            speed = self.min_speed if abs(obstacle_steer) > 0.6 else 1.0
        else:
            steering = wall_steer
            speed = self.min_speed if is_corner else 1.0

        return {"steering": steering, "speed": speed, "is_corner": is_corner}
