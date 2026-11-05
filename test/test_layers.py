"""layers: the package's one home for the strict read, the deep merge, nesting and the launch boolean."""
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
