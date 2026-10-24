"""Managed navigation adopts a complete foreign union through inert launch seams."""
import json
from pathlib import Path

from mhseals_config import Artifact, BaselineRevision, digest
from mhseals_config._bootstrap import begin_bootstrap, finish_bootstrap, read_bootstrap_inputs
from mhseals_config._evidence import Recipe
from mhseals_nav.startup_snapshot import EVALUATORS, observe_inputs, prepare_inputs
import pytest
import yaml

from test_navigation_startup_validation import inert_navigation_factory
from test_robot_overlay import robot_factory

ROOT = Path(__file__).parents[1]


def union(root, resources):
    prepared = prepare_inputs(root, 'union', ROOT / 'config/nav2_params.yaml', None,
                              sim='false', use_lidar='false', resources=resources)
    assert not root.exists(), 'pure preparation must not publish BT files or reserve scope'
    foreign = {'hardware-baseline': {'value': 1}, 'hardware-pure': {'value': 2}}
    documents = dict(prepared['documents'], **foreign)
    recipes = dict(prepared['recipes'], **{p: 'foreign/v1' for p in foreign})
    identities = dict(prepared['identities'], **{p: 'foreign-code/v1' for p in foreign})
    artifacts = prepared['baseline_artifacts'] + (
        Artifact('hardware-baseline.json', digest(foreign['hardware-baseline'])),)
    baseline = BaselineRevision(digest([a.sha256 for a in artifacts]), artifacts)
    evaluators = dict(EVALUATORS, **{
        'foreign/v1': Recipe('foreign/v1', lambda document, evidence: evidence == document)})
    ticket = begin_bootstrap(root, 'union', baseline, documents, recipes,
                             source_identity='aggregate', participant_identities=identities,
                             evaluators=evaluators, closure='inert')
    captured = read_bootstrap_inputs(root, ticket)
    observations = observe_inputs(captured.documents, ticket, 'aggregate')
    observations.update({p: {'scope': ticket.scope, 'candidate': ticket.candidate,
                             'source': 'aggregate', 'identity': identities[p],
                             'recipe': recipes[p], 'evidence': document}
                         for p, document in foreign.items()})
    assert finish_bootstrap(root, ticket, observations,
                            evaluators=evaluators)['status'] == 'verified_startup'
    return ticket, prepared['documents']['nav2-compiler']


@pytest.mark.parametrize('fault', [None, 'foreign', 'receipt', 'missing', 'wrong'])
def test_managed_real_navigation_factory(tmp_path, fault):
    from mhseals_nav.startup_projection import resource_paths
    factory = inert_navigation_factory()
    root = tmp_path / 'snapshots'
    ticket, document = union(root, resource_paths(factory['share']))
    if fault == 'foreign':
        (root / 'union/initializing/hardware-pure.json').write_text('{}')
    elif fault == 'receipt':
        path = root / 'union/initializing/receipt.json'
        receipt = json.loads(path.read_text())
        receipt['verified'].remove('hardware-pure')
        path.write_text(json.dumps(receipt))
    elif fault == 'missing':
        (root / 'union/active.json').unlink()
    context = {'params_file': '/absent/authoring.yaml', 'limits_file': '',
               'config_root': str(root), 'config_scope': 'union',
               'config_candidate': 'wrong' if fault == 'wrong' else ticket.candidate,
               'sim': 'true', 'use_lidar': 'true'}
    calls = []
    factory['Node'] = lambda **kw: calls.append(kw) or kw
    if fault:
        with pytest.raises((ValueError, FileNotFoundError)):
            factory['navigation_nodes'](context)
        assert calls == []
        assert not (root / 'union/nav2-bt').exists()
        return
    nodes = factory['navigation_nodes'](context)
    offered = yaml.safe_load(Path(nodes[0]['parameters'][0]).read_text())
    assert offered == document['parameters']
    assert offered['controller_server']['ros__parameters']['use_sim_time'] is False
    assert document['invocation'] == {'sim': False, 'use_lidar': False}
    for resource in document['resources'].values():
        if 'content_hex' in resource:
            assert Path(resource['artifact']).read_bytes() == bytes.fromhex(resource['content_hex'])


def test_managed_absence_does_not_bootstrap(tmp_path):
    factory = inert_navigation_factory()
    root = tmp_path / 'absent'
    with pytest.raises(FileNotFoundError):
        factory['navigation_nodes']({'params_file': str(ROOT / 'config/nav2_params.yaml'),
                                     'limits_file': '', 'config_root': str(root),
                                     'config_scope': 'absent', 'config_candidate': 'exact'})
    assert not (root / 'absent/initializing').exists()
    assert not (root / 'absent/active.json').exists()


def test_robot_forwards_complete_selection():
    entities = robot_factory()
    declared = {e.args[0]: e.default_value for e in entities if hasattr(e, 'default_value')}
    assert declared['config_candidate'] == ''
    timer = next(e for e in entities if getattr(e, 'period', None) == 5.0)
    arguments = dict(timer.actions[0].launch_arguments)
    context = {'config_root': '/private/store', 'config_scope': 'union',
               'config_candidate': 'exact'}
    assert {key: arguments[key].perform(context) for key in context} == context
