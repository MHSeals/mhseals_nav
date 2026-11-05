"""layers: the one home of the strict read, deep merge, nesting, launch boolean and paths."""
from pathlib import Path
import re

from mhseals_nav.layers import deep_merge, nested

ROOT = Path(__file__).resolve().parents[1]


def test_deep_merge_copies_into_dst_and_replaces_lists_scalars_and_non_mappings():
    src = {'a': {'b': [1]}, 'c': {'d': 1}}
    out = deep_merge({'a': {'x': 0}, 'c': 5}, src)
    assert out == {'a': {'x': 0, 'b': [1]}, 'c': {'d': 1}}
    out['a']['b'].append(2)
    assert src['a']['b'] == [1]


def test_nested_rebuilds_the_yaml_shape_from_parameter_names():
    assert nested({'a.b.c': 1, 'a.d': 2, 'e': 3}) == {'a': {'b': {'c': 1}, 'd': 2}, 'e': 3}


def test_the_generic_helpers_are_written_once():
    pattern = re.compile(r'^(def (deep_merge|nest|nested|boolean|load_strict)\(|'
                         r'class StrictOverlayLoader)', re.M)
    hits = sorted(str(path.relative_to(ROOT)) for path in
                  [*ROOT.glob('mhseals_nav/**/*.py'), *ROOT.glob('launch/*.py')]
                  if pattern.search(path.read_text()))
    assert hits == ['mhseals_nav/layers.py']


def test_package_files_come_from_the_source_tree_when_running_from_it():
    from mhseals_nav import device_profile, layers, robot_geometry, yaml_params
    from mhseals_nav.mavros_configuration import package_setup
    from mhseals_nav.tasks import params
    assert layers.package_path() == ROOT
    assert yaml_params.config_path('hull.yaml') == ROOT / 'config/hull.yaml'
    assert params.config_path('mission.yaml') == str(ROOT / 'config/tasks/mission.yaml')
    assert device_profile.share() == ROOT
    assert robot_geometry.description_dir() == ROOT / 'description'
    assert package_setup() == ROOT / 'config/mavros_setup.yaml'


def test_the_behavior_trees_are_found_beside_the_module():
    """The trees install with the Python package: no package_path, right under either install."""
    from mhseals_nav.startup_projection import bt_paths, PRECISE_TREE
    assert PRECISE_TREE.is_file()
    assert all(Path(tree).is_file() for tree in bt_paths().values())


def test_a_plain_install_reads_the_share(installed):
    from mhseals_nav import layers
    from mhseals_nav.startup_projection import camera_path, ceilings_path, hull_path
    from mhseals_nav.tasks import params
    assert layers.config_path('hull.yaml') == installed / 'config/hull.yaml'
    assert params.config_path('mission.yaml') == str(installed / 'config/tasks/mission.yaml')
    assert hull_path() == installed / 'config/hull.yaml'
    assert ceilings_path() == installed / 'config/nav2_ceilings.yaml'
    assert camera_path() == installed / 'config/camera.yaml'


def test_only_package_path_asks_ament_for_this_package():
    share = re.compile(r'get_package_share_directory\(\s*[\'"]mhseals_nav[\'"]')
    hits = sorted(str(path.relative_to(ROOT)) for path in
                  [*ROOT.glob('mhseals_nav/**/*.py'), *ROOT.glob('launch/*.py')]
                  if share.search(path.read_text()))
    assert hits == ['mhseals_nav/layers.py']


def test_only_layers_looks_beside_the_module_for_package_files():
    """A plain install has no config/ beside the module: files come from layers.package_path."""
    beside = re.compile(r'parents\[1\]')
    hits = sorted(str(path.relative_to(ROOT)) for path in
                  [*ROOT.glob('mhseals_nav/**/*.py'), *ROOT.glob('launch/*.py')]
                  if beside.search(path.read_text()))
    assert hits == ['mhseals_nav/layers.py']
