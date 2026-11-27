"""device_parts [DEVICE | --devices]: the parts device.launch.py can start, one per line: ``part<TAB>what it does``.

--devices prints the device profiles instead: ``device<TAB>its parts``.

With DEVICE, that profile's enabled parts (all layers, as device.launch.py resolves them); without,
every profile's, each part once per way it starts, with the devices that start it so. ``only:=`` takes
these names. What it does is the part's ``about``, then the launch file and arguments (or ros2 run).
"""

import sys

from mhseals_nav import device_profile


def starts(spec):
    """How a part starts: its launch file and its own arguments, or its ros2 run command."""
    if 'run' in spec:
        return 'ros2 run ' + ' '.join(spec['run'])
    return ' '.join([spec['launch'], *(f'{key}:={value}' for key, value in
                                       device_profile.launch_arguments({}, spec).items())])


def describe(spec, devices=()):
    where = '; '.join(filter(None, [', '.join(devices), starts(spec)]))
    return ' '.join(filter(None, [spec.get('about', ''), f'({where})']))


def device_lines():
    """One line per device profile: its enabled parts, in start order."""
    return [f"{name}\t{', '.join(part for part, _ in device_profile.select(device_profile.load(name)))}"
            for name in device_profile.available()]


def lines(device=None):
    if device == '--devices':
        return device_lines()
    if device:
        return [f'{name}\t{describe(spec)}'
                for name, spec in device_profile.select(device_profile.load(device))]
    found = {}
    for name in device_profile.available():
        for part, spec in device_profile.select(device_profile.load(name)):
            found.setdefault((part, starts(spec)), [spec, []])[1].append(name)
    return [f'{part}\t{describe(spec, devices)}' for (part, _), (spec, devices) in found.items()]


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
