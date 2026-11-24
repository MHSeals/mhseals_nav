"""`rtk base` (run the corrections source) and `rtk status` (what the rover and the source are doing).

  ros2 run mhseals_nav rtk base [PORT] [--source auto|local|ntrip] [--pin MOUNT] [--survey | --known NAME | --resume]
  ros2 run mhseals_nav rtk status [--watch]  # exit 0 only while corrections reach a subscriber

rtk base prints a status line (base_line) when it changes, at most every few seconds; `rtk status`
prints the same with the rover's fix and the survey's progress.
"""

import argparse
import re
import sys
import time

# GPSRAW fix_type: 0-1 none, 2 2D, 3 3D, 4 DGPS, 5 RTK float, 6 RTK fixed.
FIX_NAMES = {0: 'none', 1: 'none', 2: '2D', 3: '3D', 4: 'DGPS', 5: 'float', 6: 'fixed'}
SOURCES = ('auto', 'local', 'ntrip')


def rtk_fix(fix_type):
    """The RTK state of a GPSRAW fix_type: 'fixed', 'float' or 'none'."""
    return {5: 'float', 6: 'fixed'}.get(int(fix_type), 'none')


def _clock(seconds):
    seconds = int(seconds or 0)
    return f'{seconds // 60}:{seconds % 60:02d}'


def _plural(n, word):
    return f"{n} {word}{'' if n == 1 else 's'}"


def base_line(status, percent=False):
    """What the corrections source is doing and where its RTCM goes, from the /rtk/status document.

    e.g. ``survey 3:12  acc 1.84 m (target 2.0)  sats 26  -> sending RTCM 1005/1077 to
    /mavros/gps_rtk/send_rtcm (1 subscriber)``; when nothing is sent, the reason instead.
    ``percent`` adds the survey's progress (rtk status).
    """
    if not status:
        return 'rtk base not running (no /rtk/status)'
    source = status.get('source') or {}
    net = source.get('ntrip') or {}
    kind, phase, detail = source.get('kind', 'base'), status.get('phase'), status.get('detail') or ''
    parts, reason = [], ''
    if kind == 'ntrip':
        if net.get('mount'):
            parts.append(f"ntrip {net.get('caster') or '?'}/{net['mount']}"
                         + (f" {net['km']} km" if net.get('km') is not None else ''))
        else:
            parts.append('ntrip')
        reason = detail
    else:
        svin = status.get('svin') or {}
        model = status.get('base_model') or 'base'
        if phase in ('surveying', 'ready') and svin:
            parts.append(('survey done ' if phase == 'ready' else 'survey ') + _clock(svin.get('seconds'))
                         + (f" {round(100 * (status.get('survey') or 0))}%" if percent else '')
                         + (f"  acc {svin['accuracy']:.2f} m (target {status.get('survey_target')})"
                            if svin.get('accuracy') is not None else ''))
            reason = '' if phase == 'ready' else 'survey not done yet'
        elif phase == 'fixed':
            parts.append(f"{model} fixed at {status['site']}" if status.get('site') else f'{model} fixed position')
        elif phase == 'surveying':
            parts.append(f'{model} surveying')
            reason = detail
        else:
            parts.append(f'{model} {phase or "off"}')
            reason = detail
        if status.get('satellites') is not None:
            parts.append(f"sats {status['satellites']}")
    topic, subs, age = status.get('topic') or 'the RTCM topic', status.get('subscribers'), status.get('age')
    listeners = '' if subs is None else f' ({_plural(subs, "subscriber")})' if subs else \
        ' (no subscriber: is MAVROS running?)'
    if age is not None and age <= 3:
        ids = '/'.join(str(i) for i in status.get('rtcm') or []) or '?'
        parts.append(f'-> sending RTCM {ids} to {topic}{listeners}')
    else:
        if subs == 0:
            reason = '; '.join(filter(None, (reason, f'no subscriber on {topic}')))
        if not reason and net.get('detail') and kind != 'ntrip':
            reason = 'ntrip: ' + net['detail']
        parts.append('-> not sending: ' + (reason or ('no frames from the receiver' if phase in ('ready', 'fixed')
                                                       else 'no source')))
    return '  '.join(parts)


class Printer:
    """Decides when rtk base prints its line: on a change of what it says (numbers aside) at most
    every ``gap`` s, and a changed number at most every ``every`` s."""

    def __init__(self, gap=3.0, every=15.0):
        self.gap, self.every = gap, every
        self.line, self.key, self.at = None, None, float('-inf')

    def offer(self, line, now):
        """The line to print now, or None."""
        key = re.sub(r'\d+(\.\d+)?', '#', line)
        if line == self.line or now - self.at < (self.gap if key != self.key else self.every):
            return None
        self.line, self.key, self.at = line, key, now
        return line


def healthy(status):
    """Whether corrections reach the FCU: RTCM under 3 s old on a topic someone subscribes to, from a
    surveyed or fixed local base or a streaming NTRIP mountpoint (`rtk status` exits 0 only then)."""
    if not status or status.get('age') is None or status['age'] > 3 or status.get('subscribers') == 0:
        return False
    source = status.get('source') or {}
    if source.get('kind') == 'ntrip':
        return (source.get('ntrip') or {}).get('phase') == 'streaming'
    return status.get('phase') in ('ready', 'fixed')


def status_line(status, rover):
    """`rtk status`: the rover's fix (GPSRAW, may be None), then base_line with the survey's progress."""
    if rover:
        fix = rover['fix_type']
        parts = [f"rover {rtk_fix(fix)} ({FIX_NAMES.get(fix, fix)}) sats {rover['satellites_visible']}"
                 + (f" h_acc {rover['h_acc']:.2f}m" if rover.get('h_acc') is not None else '')]
    else:
        parts = ['rover no GPSRAW (is MAVROS up?)']
    if status:
        age = status.get('age')
        parts.append(f'corrections {base_line(status, percent=True)}  age ' + ('-' if age is None else f'{age:.1f}s'))
    else:
        parts.append(base_line(status))
    return ' | '.join(parts)


def parse(argv):
    """argv -> (config, survey, position, source, pin); ValueError on a bad config or an unknown position."""
    from mhseals_nav.rtk import base, ntrip
    ap = argparse.ArgumentParser(prog='rtk base', description='RTK corrections to MAVROS from a USB base receiver '
                                 'and/or public NTRIP (mountpoint picked by distance from the rough fix).')
    ap.add_argument('port', nargs='?', help='serial device of the base receiver (default: config device)')
    ap.add_argument('--config', help='settings YAML (default: $MHSEALS_RTK_CONFIG or ~/.config/mhseals/rtk.yaml)')
    ap.add_argument('--source', choices=SOURCES, default='auto',
                    help='auto: base and NTRIP, the nearer/fixed one feeds the FCU; local: the USB base only; '
                         'ntrip: NTRIP only')
    ap.add_argument('--pin', metavar='MOUNT', help='NTRIP: stream only this mountpoint (MOUNT or CASTER/MOUNT)')
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument('--survey', action='store_true', help='start a new survey-in')
    mode.add_argument('--known', metavar='NAME', help='fix the base at a named position from the config')
    mode.add_argument('--resume', action='store_true',
                      help='adopt the receiver as it is; without it an idle receiver is fixed at the nearest '
                           'saved site, else surveyed (auto_site)')
    args = ap.parse_args(argv)
    config = base.load_config(args.config)
    if config['source'] != 'ours':
        raise ValueError(f"source is {config['source']} in the settings; two correction sources "
                         'mix bases at the FCU (set source: ours to run this base)')
    if args.port:
        config['device'] = args.port
    elif not config['device'] and args.source != 'ntrip':
        device, how = base.find_receiver()
        config['device'] = device or ''
        print(f'rtk base: base receiver {device} ({how})' if device else f'rtk base: {how}', flush=True)
    if args.resume:
        config['auto_site'] = False
    position = None
    if args.known:
        if args.known not in config['positions']:
            raise ValueError(f'no known position {args.known!r}; have: ' + (', '.join(config['positions']) or 'none'))
        position = dict(config['positions'][args.known], name=args.known)
    settings = ntrip.load_settings(config)
    has_ntrip = settings['enabled'] and bool(settings['casters'])
    if args.source == 'local' and not config['device']:
        raise ValueError('--source local: no base receiver (pass a port or set device in the settings)')
    if args.source == 'ntrip' and not has_ntrip:
        raise ValueError('--source ntrip: NTRIP is off in the settings')
    if not config['device'] and not has_ntrip:
        raise ValueError('no base receiver: pass a port or set device in the settings (NTRIP is off)')
    return config, args.survey, position, args.source, args.pin


def base_main(argv=None):
    """Run the corrections source until interrupted."""
    try:
        config, survey, position, source, pin = parse(sys.argv[1:] if argv is None else argv)
    except ValueError as error:
        raise SystemExit(f'rtk base: {error}')
    from mhseals_nav.rtk.node import serve
    serve(config, survey, position, source, pin)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv and argv[0] == 'base':
        return base_main(argv[1:])
    ap = argparse.ArgumentParser(prog='rtk', description='RTK: `rtk status` (rover fix and corrections), '
                                 '`rtk base ...` (the corrections source).')
    ap.add_argument('command', choices=['status', 'base'])
    ap.add_argument('--watch', action='store_true', help='print a line every 2 s until Ctrl-C')
    # exit 0 only when healthy(): a lake checklist's `rtk status` fails while nothing reaches the FCU
    ap.add_argument('--timeout', type=float, default=3.0, help='seconds to wait for the topics')
    args = ap.parse_args(argv)
    from mhseals_nav.rtk.node import read_status
    try:
        while True:
            status, rover = read_status(args.timeout)
            print(time.strftime('%H:%M:%S') + '  ' + status_line(status, rover), flush=True)
            if not args.watch:
                return 0 if healthy(status) else 1
            time.sleep(2)
    except KeyboardInterrupt:
        return 0


if __name__ == '__main__':
    sys.exit(main())
