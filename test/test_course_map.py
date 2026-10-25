"""Course map JSON: atomic round trip through fake toLL/fromLL, age filter."""

from datetime import datetime, timedelta, timezone

from tracker_fixtures import CFG
from mhseals_nav import course_map
from mhseals_nav.track_filter import MOTION_STATIC, SOURCE_PRIOR, TrackFilter

ORIGIN = (42.0, -71.0)
M_PER_DEG = 111_320.0


def to_ll(x, y):
    return ORIGIN[0] + y / M_PER_DEG, ORIGIN[1] + x / M_PER_DEG


def from_ll(lat, lon):
    return (lon - ORIGIN[1]) * M_PER_DEG, (lat - ORIGIN[0]) * M_PER_DEG


def test_round_trip_through_lat_lon(tmp_path, monkeypatch):
    monkeypatch.setenv('MHSEALS_DATA', str(tmp_path))
    now = datetime(2026, 10, 25, 12, tzinfo=timezone.utc)
    file = course_map.path('boat')
    assert file == tmp_path / 'course_map/boat.json'
    saved = [(3.0, 4.0, 'red_buoy'), (-2.0, 7.5, 'green_buoy')]
    entries = []
    for x, y, cls in saved:
        lat, lon = to_ll(x, y)
        entries.append({'class_id': cls, 'lat': lat, 'lon': lon, 'sigma_m': 0.2, 'hits': 40,
                        'last_seen': course_map.iso(now)})
    course_map.save(file, 'boat', entries, now)
    assert not list(file.parent.glob('*.tmp'))
    loaded = course_map.load(file, now + timedelta(days=1), CFG['course_map']['max_age_days'])
    f = TrackFilter(CFG)
    for row in loaded:
        f.add_prior(row['class_id'], from_ll(row['lat'], row['lon']), row['sigma_m'], 0.0)
    got = sorted((round(t.x[0], 3), round(t.x[1], 3), t.class_id) for t in f.tracks)
    assert got == sorted(saved)
    assert {(t.motion, t.source, t.hits) for t in f.tracks} == {(MOTION_STATIC, SOURCE_PRIOR, 0)}


def test_old_entries_and_bad_files_are_ignored(tmp_path):
    now = datetime(2026, 10, 25, tzinfo=timezone.utc)
    file = tmp_path / 'm.json'
    old = {'class_id': 'red_buoy', 'lat': 1.0, 'lon': 2.0, 'sigma_m': 0.2, 'hits': 30,
           'last_seen': course_map.iso(now - timedelta(days=4))}
    course_map.save(file, 'boat', [old], now)
    assert course_map.load(file, now, 3) == []
    file.write_text('{not json')
    assert course_map.load(file, now, 3) == []
    assert course_map.load(tmp_path / 'missing.json', now, 3) == []
