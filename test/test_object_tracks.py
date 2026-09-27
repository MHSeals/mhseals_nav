"""Pure association/expiry regressions; no ROS network or hardware."""

import pytest

from mhseals_nav.object_tracks import ObjectTracks


def test_confirmation_is_single_track_and_class_aware():
    tracks = ObjectTracks()
    for stamp in [1.0, 1.2, 1.4, 1.6, 1.8]:
        tracks.update([([5.0, 0.0, 0.0], "red")], stamp)
    assert len(tracks.objects) == len(tracks.tracks) == 1
    tracks.update([([5.0, 0.0, 0.0], "green")], 2.0)
    assert len(tracks.tracks) == 2
    assert tracks.objects[0].last_seen == 1.8


def test_empty_input_silence_and_clock_reset_expire_tracks():
    tracks = ObjectTracks(confirmation_count=1, confirmation_time=0)
    tracks.update([([1.0, 2.0, 3.0], "buoy")], 10.0)
    tracks.expire(11.0)
    assert len(tracks.objects) == 1
    tracks.update([], 12.0)
    tracks.expire(14.0)
    assert not tracks.tracks
    tracks.update([([1.0, 2.0, 3.0], "buoy")], 15.0)
    tracks.expire(16.0)
    tracks.expire(1.0)
    assert not tracks.tracks
    tracks.update([([1.0, 2.0, 3.0], "buoy")], 2.0)
    assert len(tracks.objects) == 1


def test_stale_duplicates_invalid_positions_and_one_to_one_matching():
    tracks = ObjectTracks()
    tracks.update([([0.0, 0.0, 0.0], "red")], 1.0)
    tracks.update([([100.0, 0.0, 0.0], "red")], 1.0)
    assert tracks.tracks[0].hits == 1
    tracks.update([([float("nan"), 0.0, 0.0], "red")], 2.0)
    assert tracks.tracks[0].hits == 1
    tracks.update([([0.1, 0.0, 0.0], "red"), ([0.2, 0.0, 0.0], "red")], 3.0)
    assert sorted(t.hits for t in tracks.tracks) == [1, 2]
    assert tracks.tracks[0].position == pytest.approx([0.05, 0.0, 0.0])
