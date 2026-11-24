"""rtk_base (run the corrections source) and `rtk status` (what the rover and the source are doing).

  ros2 run mhseals_nav rtk_base [PORT] [--source auto|local|ntrip] [--pin MOUNT] [--survey | --known NAME | --resume]
  ros2 run mhseals_nav rtk status [--watch]
  ros2 run mhseals_nav rtk base ...          # same as rtk_base
"""

import argparse
import sys
import time

# GPSRAW fix_type: 0-1 none, 2 2D, 3 3D, 4 DGPS, 5 RTK float, 6 RTK fixed.
FIX_NAMES = {0: 'none', 1: 'none', 2: '2D', 3: '3D', 4: 'DGPS', 5: 'float', 6: 'fixed'}
SOURCES = ('auto', 'local', 'ntrip')


def rtk_fix(fix_type):
    """The RTK state of a GPSRAW fix_type: 'fixed', 'float' or 'none'."""
    return {5: 'float', 6: 'fixed'}.get(int(fix_type), 'none')


def status_line(status, rover):
    """One line from the /rtk/status document and the rover's GPSRAW fields (either may be None)."""
    if rover:
        fix = rover['fix_type']
        parts = [f"rover {rtk_fix(fix)} ({FIX_NAMES.get(fix, fix)}) sats {rover['satellites_visible']}"
                 + (f" h_acc {rover['h_acc']:.2f}m" if rover.get('h_acc') is not None else '')]
    else:
        parts = ['rover no GPSRAW (is MAVROS up?)']
    if status:
        source = status.get('source') or {}
        net = source.get('ntrip') or {}
        kind = source.get('kind', 'base')
        if kind == 'ntrip':
            where = f"ntrip {net.get('caster') or '?'}/{net.get('mount') or '-'}"
            if net.get('km') is not None:
                where += f" {net['km']} km"
        else:
            where = f"local {status.get('base_model') or 'base'}" + (f" site {status['site']}" if status.get('site') else '')
        age = status.get('age')
        parts.append(f"corrections {where} {status.get('base', '?')} age "
                     + ('-' if age is None else f'{age:.1f}s'))
        if kind != 'ntrip' and status.get('satellites') is not None:
            parts.append(f"base sats {status['satellites']}")
        if status.get('detail'):
            parts.append(status['detail'])
    else:
        parts.append('rtk_base not running (no /rtk/status)')
    return ' | '.join(parts)


def parse(argv):
    """argv -> (config, survey, position, source, pin); ValueError on a bad config or an unknown position."""
    from mhseals_nav.rtk import base, ntrip
    ap = argparse.ArgumentParser(prog='rtk_base', description='RTK corrections to MAVROS from a USB base receiver '
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
        raise SystemExit(f'rtk_base: {error}')
    from mhseals_nav.rtk.node import serve
    serve(config, survey, position, source, pin)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv and argv[0] == 'base':
        return base_main(argv[1:])
    ap = argparse.ArgumentParser(prog='rtk', description='RTK: `rtk status` (rover fix and corrections), '
                                 '`rtk base ...` (= rtk_base).')
    ap.add_argument('command', choices=['status', 'base'])
    ap.add_argument('--watch', action='store_true', help='print a line every 2 s until Ctrl-C')
    ap.add_argument('--timeout', type=float, default=3.0, help='seconds to wait for the topics')
    args = ap.parse_args(argv)
    from mhseals_nav.rtk.node import read_status
    try:
        while True:
            print(time.strftime('%H:%M:%S') + '  ' + status_line(*read_status(args.timeout)), flush=True)
            if not args.watch:
                return 0
            time.sleep(2)
    except KeyboardInterrupt:
        return 0


if __name__ == '__main__':
    sys.exit(main())
