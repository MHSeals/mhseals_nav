"""RTK: the NTRIP mount picker is distance-ranked anywhere (Texas, Florida), a pin wins, and the
rover's fix type reads as none/float/fixed in `rtk status`."""
from mhseals_nav.rtk import ntrip
from mhseals_nav.rtk.cli import rtk_fix, status_line

MSM7 = '1005(10),1077(1),1087(1),1097(1),1127(1),1230(10)'


def table(*sites):
    """A sourcetable body: one single-base, dual-band MSM7 STR record per (mount, lat, lon)."""
    lines = [f'STR;{m};{m};RTCM 3.3;{MSM7};2;GPS+GLO+GAL+BDS;NET;USA;{lat};{lon};0;0;gen;none;B;N;9600;'
             for m, lat, lon in sites]
    return '\n'.join(lines + ['ENDSOURCETABLE'])


TEXAS = table(('HOUSTON', 29.76, -95.37), ('AUSTIN', 30.27, -97.74), ('DALLAS', 32.78, -96.80))
FLORIDA = table(('MELBOURNE', 28.08, -80.61), ('MIAMI', 25.76, -80.19), ('ORLANDO', 28.54, -81.38))
STREAMS = ntrip.parse_sourcetable(TEXAS, 'A') + ntrip.parse_sourcetable(FLORIDA, 'B')


def best(here, **override):
    settings = {**ntrip.load_settings(), **override}
    return ntrip.rank(STREAMS, here, settings)


def test_the_nearest_mount_wins_in_texas_and_in_florida():
    assert best((29.70, -95.40))[0].stream.mount == 'HOUSTON'      # a lake near Houston
    assert best((30.40, -97.70))[0].stream.mount == 'AUSTIN'
    assert best((28.10, -80.65))[0].stream.mount == 'MELBOURNE'    # the Indian River
    ranked = best((25.80, -80.20))
    assert ranked[0].stream.mount == 'MIAMI' and ranked[0].in_range
    assert [c.stream.mount for c in ranked][-3:] == sorted(                      # Texas is far: last
        ['HOUSTON', 'AUSTIN', 'DALLAS'], key=lambda m: ntrip.distance_km((25.8, -80.2), next(
            (s.lat, s.lon) for s in STREAMS if s.mount == m)))


def test_a_pin_wins_over_distance_and_names_mount_or_caster_and_mount():
    ranked = best((28.10, -80.65), pin='DALLAS')
    assert [c.stream.mount for c in ranked] == ['DALLAS']
    assert [c.stream.mount for c in best((28.10, -80.65), pin='b/miami')] == ['MIAMI']
    assert best((28.10, -80.65), pin='A/MIAMI') == []              # wrong caster: nothing to stream


def test_fix_parsing_and_the_status_line():
    assert [rtk_fix(t) for t in range(7)] == ['none'] * 5 + ['float', 'fixed']
    status = {'base': 'streaming', 'age': 0.42, 'detail': 'B/MELBOURNE', 'satellites': None,
              'source': {'kind': 'ntrip', 'ntrip': {'caster': 'B', 'mount': 'MELBOURNE', 'km': 3.1}}}
    line = status_line(status, {'fix_type': 6, 'satellites_visible': 21, 'h_acc': 0.014})
    assert line.startswith('rover fixed (fixed) sats 21 h_acc 0.01m')
    assert 'corrections ntrip B/MELBOURNE 3.1 km streaming age 0.4s' in line
    line = status_line(None, {'fix_type': 5, 'satellites_visible': 9})
    assert 'rover float' in line and 'rtk_base not running' in line
    assert status_line(None, None).startswith('rover no GPSRAW')


def test_rtcm_frames_survive_junk_and_a_bad_crc():
    body = bytes([0x3E, 0xD0]) + bytes(17)                          # message 1005, 19 B payload
    frame = bytes([0xD3, 0x00, len(body)]) + body
    frame += ntrip.crc24q(frame).to_bytes(3, 'big')
    framer = ntrip.Framer()
    bad = frame[:-1] + bytes([frame[-1] ^ 1])
    assert framer.feed(b'junk' + bad + frame[:10]) == []
    assert framer.feed(frame[10:]) == [frame] and ntrip.frame_id(frame) == 1005
