"""Bounded, class-aware tracking in a continuous local frame (no ROS I/O)."""

from dataclasses import dataclass
import math


@dataclass
class Track:
    id: int
    position: tuple
    label: str
    first_seen: float
    last_seen: float
    hits: int = 1


class ObjectTracks:
    """Associate once per observation, confirm repeated hits, expire silence."""

    def __init__(
        self,
        distance=1.0,
        timeout=3.0,
        candidate_timeout=1.0,
        confirmation_count=3,
        confirmation_time=0.2,
        alpha=0.5,
        max_tracks=256,
    ):
        self.distance = distance
        self.timeout = timeout
        self.candidate_timeout = candidate_timeout
        self.confirmation_count = confirmation_count
        self.confirmation_time = confirmation_time
        self.alpha = alpha
        self.max_tracks = max_tracks
        self.tracks = []
        self.next_id = 0
        self.last_stamp = float("-inf")
        self.last_now = float("-inf")

    def confirmed(self, track):
        return (
            track.hits >= self.confirmation_count
            and track.last_seen - track.first_seen >= self.confirmation_time
        )

    def expire(self, now):
        """Also discard the old simulation timeline after a clock reset."""
        if now < self.last_now:
            self.tracks.clear()
            self.last_stamp = float("-inf")
        self.last_now = now
        self.tracks = [
            t
            for t in self.tracks
            if now - t.last_seen
            <= (self.timeout if self.confirmed(t) else self.candidate_timeout)
        ]

    def update(self, observations, stamp):
        if stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        observations = [
            (tuple(float(v) for v in p), label)
            for p, label in observations[: self.max_tracks]
            if len(p) == 3 and all(map(math.isfinite, p))
        ]
        used, matched = set(), set()
        pairs = []
        for i, track in enumerate(self.tracks):
            for j, (position, label) in enumerate(observations):
                distance = math.dist(track.position, position)
                if label == track.label and distance <= self.distance:
                    pairs.append((distance, i, j))
        # Global nearest-first, deterministic ties, one observation per track.
        # This is a short-range buoy tracker, not a multi-target motion predictor.
        for _, i, j in sorted(pairs):
            if i in matched or j in used:
                continue
            track = self.tracks[i]
            track.position = tuple(
                self.alpha * new + (1 - self.alpha) * old
                for new, old in zip(observations[j][0], track.position)
            )
            track.last_seen = stamp
            track.hits += 1
            used.add(j)
            matched.add(i)
        for j, (position, label) in enumerate(observations):
            if j not in used and len(self.tracks) < self.max_tracks:
                self.tracks.append(Track(self.next_id, position, label, stamp, stamp))
                self.next_id += 1

    @property
    def objects(self):
        return [t for t in self.tracks if self.confirmed(t)]
