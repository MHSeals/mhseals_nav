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
    read_bootstrap_inputs, read_selected_inputs,
)
from mhseals_config._evidence import Recipe
from mhseals_nav.navigation_config import (
    StrictOverlayLoader, validated_merge_limits,
)
import yaml

RECIPE = 'nav-compiler-serialization/1'
PARTICIPANT = 'nav2-compiler'


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
    from mhseals_nav import navigation_config
    return digest({str(Path(p).name): hashlib.sha256(Path(p).read_bytes()).hexdigest()
                   for p in (__file__, navigation_config.__file__)})


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


def select_snapshot(root, scope, params_file, limits_file, *, sim, use_lidar):
    """Adopt an exact scope or bootstrap a fresh, domain-validated selection."""
    def mutable(value):
        from collections.abc import Mapping
        if isinstance(value, Mapping):
            return {key: mutable(item) for key, item in value.items()}
        if isinstance(value, tuple):
            return [mutable(item) for item in value]
        return value

    try:
        selected = read_selected_inputs(root, scope, evaluators=EVALUATORS)
    except NoActiveSelection:
        pass
    else:
        document = mutable(selected.documents[PARTICIPANT])
        if document['compiler'] != compiler_identity():
            raise ValueError('navigation compiler identity changed; reconcile scope')
        return document

    base, defaults = _source(params_file)
    limits, operator = _source(limits_file, optional=True) if limits_file else (
        {}, {'path': None, 'sha256': None, 'origins': []})
    parameters = validated_merge_limits(base, limits) if limits else base
    # B contains persistent inputs only. Invocation flags belong to S.
    baseline = {'parameters': parameters, 'sources': [defaults, operator]}
    revision = digest(baseline)
    document = {'parameters': parameters, 'baseline': baseline,
                'invocation': {'sim': _boolean(sim), 'use_lidar': _boolean(use_lidar)},
                'compiler': compiler_identity(), 'scope': scope,
                'verification': 'pure-compiler-output; ROS unobserved'}
    documents = {PARTICIPANT: document, 'nav2-baseline': baseline}
    source = digest(baseline['sources'])
    identity = document['compiler']
    ticket = begin_bootstrap(
        root, scope, BaselineRevision(revision, (
            Artifact('nav2-baseline.json', digest(baseline)),)),
        documents, {name: RECIPE for name in documents},
        source_identity=source,
        participant_identities={name: identity for name in documents},
        closure='inert', evaluators=EVALUATORS)
    candidate = read_bootstrap_inputs(root, ticket)
    if mutable(candidate.documents) != documents:
        raise ValueError('materialized navigation inputs differ')
    observation = {'scope': scope, 'candidate': ticket.candidate, 'source': source,
                   'identity': identity, 'recipe': RECIPE,
                   'evidence': {'yaml': yaml.safe_dump(parameters),
                                'kind': 'pure-compiler-output'}}
    receipt = finish_bootstrap(root, ticket,
                               {name: observation for name in documents},
                               evaluators=EVALUATORS)
    if receipt['status'] != 'verified_startup':
        raise ValueError('navigation compiler serialization unverified')
    read_selected_inputs(root, scope, evaluators=EVALUATORS)
    return document
