"""
Ticketed bootstrap selections for the pure Nav2 startup compiler.

This verifies serialized compiler output, never ROS consumption. Until the
baseline publisher lands, new scopes explicitly bootstrap selected authoring
inputs; existing scopes adopt their recorded selection without rereading them.
"""
import hashlib
import os
from pathlib import Path
import uuid

from mhseals_config import Artifact, BaselineRevision, digest
from mhseals_config._bootstrap import (
    begin_bootstrap, finish_bootstrap, NoActiveSelection,
    read_bootstrap_inputs, read_selected_inputs, read_selected_subset,
)
from mhseals_config._evidence import Recipe
from mhseals_nav.bt_artifacts import materialize_trees
from mhseals_nav.navigation_config import (
    StrictOverlayLoader, validate_navigation_base, validated_merge_limits,
)
from mhseals_nav.startup_projection import (
    project_launch, resource_identity, resource_paths,
)
import yaml

RECIPE = 'nav-compiler-serialization/1'
PARTICIPANT = 'nav2-compiler'
PARTICIPANTS = frozenset(('nav2-baseline', PARTICIPANT))


def _boolean(value):
    value = str(value).strip().lower()
    if value not in ('true', '1', 'false', '0'):
        raise ValueError('invalid launch boolean: ' + value)
    return value in ('true', '1')


def snapshot_root():
    """Return the local bootstrap artifact root, separate from authoring files."""
    data = Path(os.environ.get(
        'MHSEALS_DATA', Path.home() / '.local/share/mhseals'))
    return str(Path(os.environ.get('MHSEALS_CONFIG_ROOT',
                                   data / 'config-startup')))


def snapshot_scope():
    """Allocate a fresh session unless the caller explicitly adopts one."""
    return os.environ.get('MHSEALS_CONFIG_SCOPE') or uuid.uuid4().hex


def compiler_identity():
    """Pin both the domain compiler and this startup adapter."""
    from mhseals_nav import bt_artifacts, navigation_config, startup_projection
    return digest({str(Path(p).name): hashlib.sha256(Path(p).read_bytes()).hexdigest()
                   for p in (__file__, navigation_config.__file__, startup_projection.__file__,
                             bt_artifacts.__file__)})


def _source(path, *, optional=False):
    path = Path(path).expanduser().absolute()
    try:
        text = path.read_text()
    except FileNotFoundError:
        if not optional:
            raise
        return {}, {'path': str(path), 'sha256': None, 'origins': []}
    value = yaml.load(text, Loader=StrictOverlayLoader)
    if value is None and yaml.compose(text) is None:
        value = {}
    if not isinstance(value, dict):
        raise ValueError(f'Invalid configuration {path}: expected mapping')
    origins = []

    def walk(node, keys=()):
        if isinstance(node, yaml.MappingNode):
            for key, child in node.value:
                walk(child, (*keys, key.value))
        else:
            origins.append({'key': '.'.join(keys),
                            'line': node.start_mark.line + 1,
                            'column': node.start_mark.column + 1})
    node = yaml.compose(text)
    if node is not None:
        walk(node)
    return value, {'path': str(path), 'sha256': hashlib.sha256(
        text.encode()).hexdigest(), 'origins': origins}


def _serialized(document, evidence):
    # This recipe observes the actual bytes offered to the launch YAML reader.
    # Compiler correctness has separate independent expected-value fixture tests.
    return (set(evidence) == {'yaml', 'kind'}
            and evidence['kind'] == 'pure-compiler-output'
            and yaml.safe_load(evidence['yaml']) == document['parameters'])


EVALUATORS = {RECIPE: Recipe(RECIPE, _serialized)}


def _mutable(value):
    from collections.abc import Mapping
    if isinstance(value, Mapping):
        return {key: _mutable(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_mutable(item) for item in value]
    return value


def prepare_inputs(root, scope, params_file, limits_file, *, sim, use_lidar, resources=None):
    """Compile the fixed navigation closure without reserving or writing a scope."""
    paths = resources or resource_paths()
    base, defaults = _source(params_file)
    limits, operator = _source(limits_file, optional=True) if limits_file else (
        {}, {'path': None, 'sha256': None, 'origins': []})
    validate_navigation_base(base)
    parameters = validated_merge_limits(base, limits)
    # B contains persistent inputs only. Invocation flags belong to S.
    baseline = {'parameters': parameters, 'sources': [defaults, operator]}
    revision = digest(baseline)
    invocation = {'sim': _boolean(sim), 'use_lidar': _boolean(use_lidar)}
    resources = resource_identity(paths)
    for resource in resources.values():
        if 'content_hex' in resource:
            resource['artifact'] = str(Path(os.path.abspath(root)) / scope / 'nav2-bt'
                                       / (resource['sha256'] + '.xml'))
    effective, projection = project_launch(parameters, invocation, resources)
    document = {'parameters': effective, 'baseline': baseline,
                'invocation': invocation, 'resources': resources,
                'projection': projection,
                'compiler': compiler_identity(), 'scope': scope,
                'verification': 'pure-compiler-output; ROS unobserved'}
    documents = {PARTICIPANT: document, 'nav2-baseline': baseline}
    source = digest(baseline['sources'])
    identity = document['compiler']
    artifacts = (Artifact('nav2-baseline.json', digest(baseline)),)
    return {'documents': documents, 'recipes': {p: RECIPE for p in PARTICIPANTS},
            'identities': {p: identity for p in PARTICIPANTS},
            'baseline_artifacts': artifacts,
            'baseline': BaselineRevision(revision, artifacts), 'source': source}


def observe_inputs(documents, ticket, source):
    """Bind serialization observations over the owner's captured documents."""
    return {p: {'scope': ticket.scope, 'candidate': ticket.candidate, 'source': source,
                'identity': documents[PARTICIPANT]['compiler'], 'recipe': RECIPE,
                'evidence': {'yaml': yaml.safe_dump(_mutable(documents[p]['parameters'])),
                             'kind': 'pure-compiler-output'}} for p in PARTICIPANTS}


def select_snapshot(root, scope, params_file, limits_file, *, sim, use_lidar, resources=None,
                    config_candidate=None):
    """Adopt captured invocation values, or bootstrap an unmanaged domain scope."""
    if config_candidate and not scope:
        raise ValueError('managed navigation startup requires explicit scope')
    paths = resources or resource_paths()
    try:
        selected = read_selected_subset(
            root, scope, PARTICIPANTS, evaluators=EVALUATORS,
            expected_candidate=config_candidate)
    except NoActiveSelection:
        if config_candidate:
            raise
    else:
        document = _mutable(selected.documents[PARTICIPANT])
        pinned = document['resources']
        if (set(paths) != set(pinned)
                or any(str(Path(path).absolute()) != pinned[key]['path']
                       for key, path in paths.items())
                or hashlib.sha256(Path(paths['launch']).read_bytes()).hexdigest()
                != pinned['launch']['sha256']):
            raise ValueError('navigation launch resources changed; reconcile scope')
        if document['compiler'] != compiler_identity():
            raise ValueError('navigation compiler identity changed; reconcile scope')
        materialize_trees(root, scope, pinned)
        return document

    prepared = prepare_inputs(root, scope, params_file, limits_file,
                              sim=sim, use_lidar=use_lidar, resources=paths)
    ticket = begin_bootstrap(
        root, scope, prepared['baseline'], prepared['documents'], prepared['recipes'],
        source_identity=prepared['source'], participant_identities=prepared['identities'],
        closure='inert', evaluators=EVALUATORS)
    candidate = read_bootstrap_inputs(root, ticket)
    if _mutable(candidate.documents) != prepared['documents']:
        raise ValueError('materialized navigation inputs differ')
    observations = observe_inputs(candidate.documents, ticket, candidate.candidate['source'])
    receipt = finish_bootstrap(root, ticket, observations, evaluators=EVALUATORS)
    if receipt['status'] != 'verified_startup':
        raise ValueError('navigation compiler serialization unverified')
    selected = read_selected_inputs(root, scope, evaluators=EVALUATORS)
    document = _mutable(selected.documents[PARTICIPANT])
    materialize_trees(root, scope, document['resources'])
    return document
