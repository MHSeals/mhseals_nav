"""Thin adapter from either tracker interface to one track tuple the tasks read.

Today's main publishes ``/objects/map`` (vision_msgs Detection3DArray); perception v2
adds ``/perception/tracks`` (mhseals_nav/msg/TrackArray) with covariance, motion and
course-map priors. Both become ``Track`` tuples, so leaves never see a message type.
The functions take duck-typed messages, so they are testable without ROS.
"""

from collections import namedtuple
import math

# sigma: 1-sigma position uncertainty in m (None when the source gives none).
# prior: loaded from the course map and not yet seen this run (v2 only).
Track = namedtuple('Track', 'label x y confidence sigma static prior id')

MOTION_STATIC, SOURCE_PRIOR = 1, 1  # mhseals_nav/msg/Track constants


def from_detections(message):
    """Tracks from a ``/objects/map`` Detection3DArray (live tracks only, no covariance)."""
    tracks = []
    for d in message.detections:
        if not d.results:
            continue
        hypothesis = d.results[0].hypothesis
        tracks.append(Track(hypothesis.class_id, d.bbox.center.position.x,
                            d.bbox.center.position.y, float(hypothesis.score) or 1.0,
                            None, True, False, str(d.id)))
    return tracks


def from_track_array(message):
    """Tracks from a v2 ``/perception/tracks`` TrackArray, priors included."""
    tracks = []
    for t in message.tracks:
        cov = t.pose.covariance
        sigma = math.sqrt(max(cov[0] + cov[7], 0.0) / 2) if any(cov) else None
        tracks.append(Track(t.class_id, t.pose.pose.position.x, t.pose.pose.position.y,
                            float(t.confidence), sigma, t.motion == MOTION_STATIC,
                            t.source == SOURCE_PRIOR, str(t.id)))
    return tracks


def remember(memory, seen, now, horizon=5.0, separation=1.0, horizons=None, exclusive=()):
    """Merge ``seen`` tracks into ``memory`` [(Track, last_seen)], forgetting stale ones.

    A track is forgotten ``horizon`` s after it was last seen, or its label's own
    ``horizons`` entry (fixed landmarks). A sighting with a remembered track's id, or
    within ``separation`` of one of the same label, replaces it, so a track that drops
    out for a moment stays put rather than flickering. The ``exclusive`` labels are
    states of one object (a slip light's colours): a sighting of one within
    ``separation`` replaces any of them, so a light that changed colour is not also
    remembered as it was; of two in one call, the more confident stays. Priors are never
    remembered: the course map republishes them while they hold.
    """
    horizons, exclusive = horizons or {}, set(exclusive)

    def same(old, track):
        return (old.label == track.label or {old.label, track.label} <= exclusive) and (
            (track.id is not None and old.id == track.id)
            or math.dist((track.x, track.y), (old.x, old.y)) < separation)

    kept = [(track, seen_at) for track, seen_at in memory
            if now - seen_at <= horizons.get(track.label, horizon)]
    for track in seen:
        if track.prior:
            continue
        match = next((i for i, (old, _) in enumerate(kept) if same(old, track)), None)
        if match is None:
            kept.append((track, now))
        elif not (kept[match][1] == now and kept[match][0].label != track.label
                  and kept[match][0].confidence > track.confidence):
            kept[match] = (track, now)
    return kept


def objects(tracks, min_confidence=0.0):
    """Return the (label, x, y) view of live tracks the task leaves plan with."""
    return [(t.label, t.x, t.y) for t in tracks
            if not t.prior and t.confidence >= min_confidence]
