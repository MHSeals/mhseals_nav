"""device_parts [DEVICE]: the parts device.launch.py can start, one per line: ``part<TAB>what it starts``.

With DEVICE, that profile's enabled parts (all layers, as device.launch.py resolves them); without,
every profile's, each part once with the devices that have it. ``only:=`` takes these names.
"""

import sys

from mhseals_nav import device_profile


def describe(spec):
    """What a part starts: its launch file or ros2 run command, then its nodes."""
    what = spec['launch'] if 'launch' in spec else 'ros2 run ' + ' '.join(spec['run'])
    return f"{what}: {', '.join(spec['nodes'])}" if spec.get('nodes') else what


def lines(device=None):
    if device:
        return [f'{name}\t{describe(spec)}'
                for name, spec in device_profile.select(device_profile.load(device))]
    found = {}
    for name in device_profile.available():
        for part, spec in device_profile.select(device_profile.load(name)):
            found.setdefault(part, [describe(spec), []])[1].append(name)
    return [f"{part}\t{what} ({', '.join(devices)})" for part, (what, devices) in found.items()]


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) > 1 or argv[:1] in (['-h'], ['--help']):
        print(__doc__.strip())
        return 0 if argv[:1] in (['-h'], ['--help']) else 2
    try:
        print('\n'.join(lines(argv[0] if argv else None)))
    except ValueError as error:
        print(f'device_parts: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
