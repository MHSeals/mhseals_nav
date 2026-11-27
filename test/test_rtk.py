"""RTK: the NTRIP mount picker is distance-ranked anywhere (Texas, Florida) by each mount's own
position (a name or "Is near:" never makes a network), a pin wins, a refusing caster rests whole,
the F9P firmware is told from MON-VER, and the status line says what is sent or why not."""
from mhseals_nav.rtk import base, ntrip
from mhseals_nav.rtk.cli import base_line, healthy, Printer, rtk_fix, status_line

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
    status = {'base': 'streaming', 'age': 0.42, 'detail': 'B/MELBOURNE', 'satellites': None, 'subscribers': 1,
              'topic': '/mavros/gps_rtk/send_rtcm', 'rtcm': [1005, 1077],
              'source': {'kind': 'ntrip', 'ntrip': {'caster': 'B', 'mount': 'MELBOURNE', 'km': 3.1}}}
    line = status_line(status, {'fix_type': 6, 'satellites_visible': 21, 'h_acc': 0.014})
    assert line.startswith('rover fixed (fixed) sats 21 h_acc 0.01m')
    assert ('corrections ntrip B/MELBOURNE 3.1 km  -> sending RTCM 1005/1077 to /mavros/gps_rtk/send_rtcm '
            '(1 subscriber)  age 0.4s') in line
    line = status_line(None, {'fix_type': 5, 'satellites_visible': 9})
    assert 'rover float' in line and 'rtk base not running' in line
    assert status_line(None, None).startswith('rover no GPSRAW')


def test_rtcm_frames_survive_junk_and_a_bad_crc():
    body = bytes([0x3E, 0xD0]) + bytes(17)                          # message 1005, 19 B payload
    frame = bytes([0xD3, 0x00, len(body)]) + body
    frame += ntrip.crc24q(frame).to_bytes(3, 'big')
    framer = ntrip.Framer()
    bad = frame[:-1] + bytes([frame[-1] ^ 1])
    assert framer.feed(b'junk' + bad + frame[:10]) == []
    assert framer.feed(frame[10:]) == [frame] and ntrip.frame_id(frame) == 1005


def test_only_the_sourcetable_makes_a_network_mount_and_its_own_position_wins():
    # RTK2go-style records: GGA wanted, "Is near:" descriptions and NET in names, but solution 0 and
    # no NET record. Each is ranked by its own position, so Poland stays ~9000 km from Texas.
    community = '\n'.join([
        f'STR;KRAKOW_NET;Is near: Houston;RTCM 3.2;{MSM7};2;GPS+GLO+GAL+BDS;SNIP;POL;50.06;19.94;1;0;sNTRIP;none;N;N;0;',
        f'STR;NETLAKE;Is near: Lake Conroe;RTCM 3.2;{MSM7};2;GPS+GLO+GAL+BDS;SNIP;USA;30.40;-95.55;1;0;sNTRIP;none;N;N;0;',
        f'STR;COCOA;Is near: Cocoa;RTCM 3.2;{MSM7};2;GPS+GLO+GAL+BDS;SNIP;USA;28.39;-80.74;1;0;sNTRIP;none;N;N;0;',
        'ENDSOURCETABLE'])
    streams = ntrip.parse_sourcetable(community, 'C') + STREAMS
    assert not any(s.vrs for s in streams)
    settings = ntrip.load_settings()
    texas = ntrip.rank(streams, (30.35, -95.50), settings)
    assert texas[0].stream.mount == 'NETLAKE' and texas[0].km < 10
    poland = next(c for c in texas if c.stream.mount == 'KRAKOW_NET')
    assert poland.km > 8000 and not poland.in_range and poland is texas[-1]
    florida = ntrip.rank(streams, (28.10, -80.65), settings)
    assert [c.stream.mount for c in florida[:2]] == ['MELBOURNE', 'COCOA']
    # A real network: NET record + solution 1, no position of its own -> half the nearest station.
    network = '\n'.join([
        'NET;FLNET;FDOT;B;N;http://example.invalid;;;',
        f'STR;NAD_VRS;Network;RTCM 3.2;{MSM7};2;GPS+GLO+GAL+BDS;FLNET;USA;0;0;1;1;Spider;none;B;N;0;',
        f'STR;ORPHAN;Network;RTCM 3.2;{MSM7};2;GPS+GLO+GAL+BDS;OTHER;USA;0;0;1;1;Spider;none;B;N;0;',
        f'STR;MELB;Melbourne;RTCM 3.2;{MSM7};2;GPS+GLO+GAL+BDS;FLNET;USA;28.08;-80.61;0;0;Spider;none;B;N;0;',
        'ENDSOURCETABLE'])
    streams = ntrip.parse_sourcetable(network, 'F')
    assert [s.vrs for s in streams] == [True, False, False]
    ranked = {c.stream.mount: c for c in ntrip.rank(streams, (28.10, -80.65), settings)}
    assert 'ORPHAN' not in ranked          # solution 1 without a NET record and no position: unranked
    assert abs(ranked['NAD_VRS'].km - ranked['MELB'].km / 2) < 1e-6


class Caster:
    """A fake caster socket: the sourcetable for GET /, a refusal with a reason for any mountpoint."""

    def __init__(self, log):
        self.log, self.reply = log, b''

    def sendall(self, data):
        path = data.split(b' ')[1].decode()
        self.log.append(path)
        self.reply = (b'SOURCETABLE 200 OK\r\nContent-Type: text/plain\r\n\r\n' + TEXAS.encode() if path == '/' else
                      b'HTTP/1.1 403 Forbidden\r\nContent-Type: text/plain\r\n\r\nYour IP is not allowed: too many attempts\r\n')

    def recv(self, size):
        data, self.reply = self.reply, b''
        return data

    def settimeout(self, value):
        pass

    def close(self):
        pass


def test_a_refusing_caster_rests_whole_and_says_why():
    log, now = [], [100.0]
    settings = ntrip.load_settings()
    settings['casters'] = [dict(name='A', host='caster.invalid', port=2101, user_env='A_USER')]
    source = ntrip.NtripSource(settings, position=lambda: (29.70, -95.40, 0.0), env={'A_USER': 'someone'},
                               connect=lambda *args: Caster(log), clock=lambda: now[0])
    source.wanted.set()
    source.step()
    assert log == ['/', '/HOUSTON']
    assert source.resting['A'] == 100.0 + settings['auth_cooldown_s']
    source.step()                                       # no second mountpoint of a refusing caster
    assert log == ['/', '/HOUSTON']
    detail = source.snapshot()['detail']
    assert detail.startswith('A cooling down 600 s: A refused /HOUSTON: HTTP/1.1 403 Forbidden')
    assert 'not allowed: too many attempts' in detail
    now[0] += settings['auth_cooldown_s'] + 1           # rested: tried again
    source.step()
    assert log[-1] == '/HOUSTON'


def test_a_socket_timeout_while_connecting_fails_over_instead_of_ending_the_thread():
    class Silent(Caster):
        """Answers the sourcetable, then times out on a mountpoint (RTK2go under load did)."""

        def recv(self, size):
            if self.log[-1] != '/':
                raise TimeoutError('timed out')
            return Caster.recv(self, size)
    log = []
    settings = ntrip.load_settings()
    settings['casters'] = [dict(name='A', host='caster.invalid', port=2101, user_env='A_USER')]
    source = ntrip.NtripSource(settings, position=lambda: (29.70, -95.40, 0.0), env={'A_USER': 'someone'},
                               connect=lambda *args: Silent(log), clock=lambda: 100.0)
    source.wanted.set()
    source.step()                                       # raised TimeoutError out of the thread before
    assert log[-1] == '/HOUSTON'
    assert 'failover' in source.snapshot()['detail'] and 'TimeoutError' in source.snapshot()['detail']


def test_why_not_names_the_missing_position_credentials_and_pin():
    settings = ntrip.load_settings()
    settings['casters'] = [dict(name='A', host='caster.invalid', port=2101, user_env='A_USER')]
    settings['secrets'] = ['/nonexistent/secrets.env']
    here = [None]
    source = ntrip.NtripSource(settings, position=lambda: here[0], env={}, connect=lambda *a: None)
    assert source.why_not().startswith('no position yet')
    here[0] = (29.7, -95.4, 0.0)
    assert source.why_not().startswith('no sourcetable yet')
    source.tables['A'] = ntrip.parse_sourcetable(TEXAS, 'A')
    assert source.why_not().startswith('no credentials for caster A: set A_USER in /nonexistent/secrets.env')
    source.settings['pin'] = 'MIAMI'
    assert source.why_not() == 'pin MIAMI not in any sourcetable'


def test_firmware_from_mon_ver():
    l1l5 = base.receiver_info(['EXT CORE 1.00 (a1b2c3)', '00190000', 'ROM BASE 0x118B2060', 'FWVER=HPG L1L5 1.40',
                               'PROTVER=27.50', 'MOD=NEO-F9P-15B', 'GPS;GAL;BDS;QZSS', 'NAVIC'])
    assert (l1l5['model'], l1l5['bands'], l1l5['module'], l1l5['gnss']) == ('F9P', 'L1L5', 'NEO-F9P-15B', ['BDS', 'GAL', 'GPS'])
    l1l2 = base.receiver_info(['EXT CORE 1.00 (0fa0ae)', '00190000', 'FWVER=HPG 1.32', 'PROTVER=27.31',
                               'MOD=ZED-F9P', 'GPS;GLO;GAL;BDS', 'SBAS;QZSS'])
    assert (l1l2['bands'], l1l2['gnss']) == ('L1L2', ['BDS', 'GAL', 'GLO', 'GPS'])
    assert base.receiver_info(['ROM CORE 3.01', '00080000', 'FWVER=HPG 1.40REF', 'MOD=NEO-M8P-2'])['bands'] == 'L1'
    assert base.receiver_info(['ROM CORE 3.01', '00080000'])['model'] is None


def test_the_receiver_port_is_found_and_another_vendor_is_never_picked():
    class Port:
        def __init__(self, device, vid):
            self.device, self.vid = device, vid

    def paths(found):
        return lambda pattern: [p for p in found if p.startswith(pattern.split('*')[0])]

    by_id = '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00'
    assert base.find_receiver(paths([by_id, '/dev/ttyACM0']), [])[0] == by_id
    ports = [Port('/dev/ttyACM0', 0x2DAE), Port('/dev/ttyACM1', 0x1546)]
    assert base.find_receiver(paths(['/dev/ttyACM0', '/dev/ttyACM1']), ports)[0] == '/dev/ttyACM1'
    assert base.find_receiver(paths(['/dev/ttyACM0']), ports[:1])[0] is None     # an FCU, not a receiver
    found = base.receivers(paths(['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2']), ports)   # rtk ports
    assert [device for device, _ in found] == ['/dev/ttyACM1', '/dev/ttyACM2']


def test_the_base_line_shows_the_survey_and_where_rtcm_goes_or_why_not():
    status = {'phase': 'surveying', 'base_model': 'Here4', 'satellites': 26, 'survey': 0.5, 'survey_target': 2.0,
              'svin': {'seconds': 192, 'accuracy': 1.84}, 'age': 0.2, 'rtcm': [1005, 1077, 1230],
              'topic': '/mavros/gps_rtk/send_rtcm', 'subscribers': 1, 'source': {'kind': 'base'}}
    assert base_line(status) == ('survey 3:12  acc 1.84 m (target 2.0)  sats 26  '
                                 '-> sending RTCM 1005/1077/1230 to /mavros/gps_rtk/send_rtcm (1 subscriber)')
    assert base_line(status, percent=True).startswith('survey 3:12 50%  acc')
    status.update(phase='error', detail='receiver refused config (L1/L2 signals, CFG-VALSET)', age=None, subscribers=0)
    assert base_line(status).endswith('-> not sending: receiver refused config (L1/L2 signals, CFG-VALSET); '
                                      'no subscriber on /mavros/gps_rtk/send_rtcm')
    printer = Printer(gap=3, every=15)
    assert printer.offer('survey 0:01', 0) and printer.offer('survey 0:02', 1) is None
    assert printer.offer('survey 0:20', 16) == 'survey 0:20'
    assert printer.offer('fixed at dock', 17) is None and printer.offer('fixed at dock', 19.5) == 'fixed at dock'


def test_rtk_status_is_healthy_only_while_corrections_reach_a_subscriber():
    live = {'phase': 'fixed', 'age': 0.3, 'subscribers': 1, 'source': {'kind': 'base'}}
    assert healthy(live) and not healthy(None)
    assert not healthy({**live, 'subscribers': 0}) and not healthy({**live, 'age': None})
    assert not healthy({**live, 'phase': 'surveying'})
    net = {'phase': 'ready', 'age': 0.5, 'subscribers': 1, 'source': {'kind': 'ntrip', 'ntrip': {'phase': 'streaming'}}}
    assert healthy(net) and not healthy({**net, 'source': {'kind': 'ntrip', 'ntrip': {'phase': 'connecting'}}})


def test_a_caster_that_picks_the_nearest_station_ranks_at_its_assumed_distance():
    # RTKdata-style records: no position (0, 0), solution 0, GGA wanted; the caster picks the station.
    auto = '\n'.join([
        'STR;AUTO;AUTO;RTCM3.2;1005,1074,1084,1094,1124;2;G,R,E,C,J;GEODNET;;0.00;0.00;1;0;GEODNET;none;B;N;9600;none',
        'STR;AUTO_WGS84;AUTO_WGS84;RTCM3.2;1005,1074,1084,1094,1124;2;G,R,E,C,J;GEODNET;;0.00;0.00;1;0;GEODNET;none;B;N;9600;none',
        'ENDSOURCETABLE'])
    florida = ntrip.parse_sourcetable(FLORIDA, 'B')
    streams = ntrip.parse_sourcetable(auto, 'R') + florida
    settings = ntrip.load_settings()
    nearest = {'R': {'AUTO_WGS84': 20.0}}
    dallas = ntrip.rank(streams, (32.78, -96.80), settings, nearest=nearest)
    assert dallas[0].key == ('R', 'AUTO_WGS84') and dallas[0].in_range and dallas[0].km == 20.0
    assert 'AUTO' not in {c.stream.mount for c in dallas}            # only the named mountpoint is ranked
    melbourne = ntrip.rank(streams, (28.10, -80.65), settings, nearest=nearest)
    assert melbourne[0].stream.mount == 'MELBOURNE'                   # a free station in range still wins
    assert not ntrip.rank(ntrip.parse_sourcetable(auto, 'R'), (32.78, -96.80), settings)   # unnamed: unranked
    rtkdata = next(c for c in settings['casters'] if c['name'] == 'RTKdata')
    assert rtkdata['nearest'] == {'AUTO_WGS84': 20.0}
