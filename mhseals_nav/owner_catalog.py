"""Explicit complete installed nav inventory; no discovery or live authority."""
from collections.abc import Mapping
from hashlib import sha256
import json

from mhseals_config.catalog import DescriptorBundle
import yaml

from .catalog import mavros_catalog, navigation_catalog, REGISTRY
from .lidar_catalog import lidar_catalog, REGISTRY as LIDAR_REGISTRY
from .lidar_configuration import package_share
from .navigation_config import StrictOverlayLoader
from .startup_catalog import startup_catalogs, ODOMETRY_RESOURCE, LOCALIZATION_RESOURCES

FAMILIES = ('nav2', 'mavros', 'lidar', 'odometry', 'localization')


def _plain(value):
    if isinstance(value, Mapping):
        return {key: _plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_plain(item) for item in value]
    return value


def combine_catalogs(families, *, consumer_versions=None):
    """Validate all required families before union, rejecting identity conflicts."""
    if set(families) != set(FAMILIES):
        raise ValueError('complete nav2/mavros/lidar families required')
    versions = dict(consumer_versions or {})
    sources, resources, registry, pins, leaves, identities = {}, {}, {}, {}, [], set()
    revisions = []
    for family in FAMILIES:
        bundle, raw_resources, bindings = families[family]
        data = _plain(bundle.descriptor)
        if data['owner_package'] != 'mhseals_nav':
            raise ValueError('unexpected nav catalog owner')
        if not data['leaves'] or not data['default_sources']:
            raise ValueError('empty required family: ' + family)
        declared = {source['id'] for source in data['default_sources']}
        if set(raw_resources) != declared:
            raise ValueError('family resource inventory mismatch: ' + family)
        DescriptorBundle.read(data, raw_resources, bindings, versions)
        revisions.append(family + '@' + data['revision'])
        for source in data['default_sources']:
            sid = source['id']
            if sid in sources and (sources[sid] != source or
                                   resources[sid] != raw_resources[sid]):
                raise ValueError('conflicting source identity: ' + sid)
            sources[sid], resources[sid] = source, raw_resources[sid]
        for key, binding in bindings.items():
            if key in registry and registry[key] is not binding:
                raise ValueError('conflicting registry identity: ' + key)
            registry[key] = binding
        for key, version in data['consumer_versions'].items():
            if key in pins and pins[key] != version:
                raise ValueError('conflicting consumer version: ' + key)
            pins[key] = version
        for leaf in data['leaves']:
            names = {leaf['canonical_key'], *leaf['aliases']}
            if identities & names:
                raise ValueError('conflicting canonical/alias identity')
            identities.update(names)
            leaves.append(leaf)
    data = {'schema_version': 1, 'owner_package': 'mhseals_nav',
            'revision': '+'.join(revisions), 'consumer_versions': pins,
            'default_sources': list(sources.values()), 'leaves': leaves}
    identity = sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()
    data['revision'] += ':sha256:' + identity
    return DescriptorBundle.read(data, resources, registry, versions)


def _require_mapping(raw, label):
    try:
        value = yaml.load(raw, Loader=StrictOverlayLoader)
    except yaml.YAMLError as exc:
        raise ValueError('malformed catalog input: ' + label) from exc
    if not isinstance(value, Mapping):
        raise ValueError('catalog input must be a mapping: ' + label)


def load_catalog(*, nav2=None, mavros=None, consumer_versions=None):
    """
    Load required installed families, optionally selecting named Nav2/MAVROS bytes.

    Each selection is ``(package-relative identity label, caller-supplied bytes)``.
    Its label is provenance supplied by the caller, not an installed filesystem
    origin: no file is read at that label. Selected descriptor defaults describe
    those requested bytes, never observed/live values or separate runtime layers.
    Lidar, odometry and localization remain installed package inventory; runtime layer origins belong to
    the separate lidar resolver. Missing or malformed installed families raise.
    """
    share = package_share('mhseals_nav')
    policy = (share / 'config/catalog.yaml').read_bytes()
    _require_mapping(policy, 'config/catalog.yaml')
    families = {}
    for family, selection, filename, loader, bindings in (
            ('nav2', nav2, 'nav2_params.yaml', navigation_catalog, REGISTRY),
            ('mavros', mavros, 'mavros_setup.yaml', mavros_catalog, REGISTRY),
            ('lidar', None, 'lidar.yaml', lidar_catalog, LIDAR_REGISTRY)):
        resource, raw = (selection if selection is not None else
                         ('config/' + filename,
                          (share / 'config' / filename).read_bytes()))
        _require_mapping(raw, resource)
        bundle = loader(raw, resource=resource, policy_raw=policy,
                        consumer_versions=consumer_versions)
        source = bundle.descriptor['default_sources'][0]['id']
        families[family] = (bundle, {source: raw}, bindings)
    startup_resources = {path: (share / path).read_bytes()
                         for path in (ODOMETRY_RESOURCE, *LOCALIZATION_RESOURCES.values())}
    families.update(startup_catalogs(startup_resources, policy_raw=policy,
                                     consumer_versions=consumer_versions))
    return combine_catalogs(families, consumer_versions=consumer_versions)
