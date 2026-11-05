"""The package's config plumbing, free of ROS and domain logic.

The strict YAML read, the one deep merge, dotted names to nesting, the launch boolean, and the
package < deployment < operator startup layering with one identity per file read.
"""
from copy import deepcopy
from hashlib import sha256
from math import isfinite
from pathlib import Path
from types import MappingProxyType

import yaml

SOURCE = Path(__file__).resolve().parents[1]   # the source tree, when this module runs from one


def package_path(*parts):
    """This package's shipped file: the source tree when this module runs from it, else the share.

    The package's one lookup of its own files. A source checkout (or a symlink install) has
    package.xml beside this module's package; a plain install puts the module in site-packages,
    so the installed share answers.
    """
    if (SOURCE / 'package.xml').exists():
        return SOURCE.joinpath(*parts)
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('mhseals_nav')).joinpath(*parts)


def config_path(*parts):
    """config/<parts> of this package (see ``package_path``)."""
    return package_path('config', *parts)


def deep_merge(dst, src):
    """Recursively copy src onto dst in place; lists, scalars and a mapping over a non-mapping replace."""
    for key, value in src.items():
        if isinstance(value, dict) and isinstance(dst.get(key), dict):
            deep_merge(dst[key], value)
        else:
            dst[key] = deepcopy(value)
    return dst


class StrictOverlayLoader(yaml.SafeLoader):
    """Do not discard ambiguous user assignments during dictionary construction."""

    def construct_mapping(self, node, deep=False):
        result = {}
        for key_node, value_node in node.value:
            if key_node.tag == 'tag:yaml.org,2002:merge':
                raise ValueError(f'Merge keys are unsupported at {key_node.start_mark}')
            key = self.construct_object(key_node, deep=deep)
            if key in result:
                raise ValueError(f'Duplicate key {key!r} at {key_node.start_mark}')
            result[key] = self.construct_object(value_node, deep=deep)
        return result

    def compose_node(self, parent, index):
        if self.check_event(yaml.AliasEvent):
            raise ValueError('YAML aliases are unsupported in navigation overlays at '
                             f'{self.peek_event().start_mark}')
        return super().compose_node(parent, index)

    def construct_yaml_float(self, node):
        value = super().construct_yaml_float(node)
        if not isfinite(value):
            raise ValueError(f'Non-finite number at {node.start_mark}')
        return value


StrictOverlayLoader.add_constructor('tag:yaml.org,2002:float',
                                    StrictOverlayLoader.construct_yaml_float)


def load_strict(raw, path, loader=StrictOverlayLoader):
    """Strict-load YAML text or bytes; every failure names the file and position."""
    try:
        return yaml.load(raw, Loader=loader)
    except (ValueError, yaml.YAMLError) as error:
        raise ValueError(f'{path}: {error}') from error


def nested(flat):
    """{'a.b': 1} -> {'a': {'b': 1}} (ROS parameter names to the YAML shape)."""
    out = {}
    for key, value in flat.items():
        *parents, leaf = key.split('.')
        node = out
        for part in parents:
            node = node.setdefault(part, {})
        node[leaf] = value
    return out


def boolean(value):
    """Parse a launch boolean, the package's one parser: true/false/1/0 (any case) or a bool."""
    if type(value) is bool:
        return value
    if not isinstance(value, str) or value.strip().lower() not in ('true', 'false', '1', '0'):
        raise ValueError(f'invalid launch boolean: {value}')
    return value.strip().lower() in ('true', '1')


def read(selected):
    """Return the selected file's resolved path, bytes and sha256 (a missing file raises)."""
    path = Path(selected).expanduser().resolve(strict=True)
    raw = path.read_bytes()
    return path, raw, sha256(raw).hexdigest()


def resolve(package, deployment='', operator='', *, validate):
    """Merge the selected layers in order, each validated alone by ``validate(layer, label)``.

    A validate that returns a mapping hands back the layer's normalized form, which is merged
    instead. An empty selection is skipped. Returns the merged values and one frozen
    {path, sha256, layer} identity per file read.
    """
    values, inputs = {}, []
    for label, selected in (('package', package), ('deployment', deployment),
                            ('operator', operator)):
        if not selected:
            continue
        path, raw, digest = read(selected)
        layer = load_strict(raw, path)
        normalized = validate(layer, label)
        deep_merge(values, layer if normalized is None else normalized)
        inputs.append(MappingProxyType({'path': str(path), 'sha256': digest, 'layer': label}))
    return values, inputs
