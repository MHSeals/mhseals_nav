"""One strict launch boolean parser (layers.boolean); the launch harness tests run its callers."""
from pathlib import Path

import pytest


@pytest.mark.parametrize('text, value', [('1', True), ('True', True), ('0', False),
                                         ('false', False)])
def test_one_strict_parser(text, value):
    from mhseals_nav.layers import boolean
    assert boolean(text) is value


@pytest.mark.parametrize('text', ['yes', '', 'on', None])
def test_one_strict_parser_rejects(text):
    from mhseals_nav.layers import boolean
    with pytest.raises(ValueError, match='invalid launch boolean'):
        boolean(text)


def test_no_other_launch_boolean_parser():
    root = Path(__file__).parents[1]
    for path in [*root.glob('launch/*.py'), *root.glob('mhseals_nav/*.py')]:
        text = path.read_text()
        assert "== 'true'" not in text and 'def launch_boolean' not in text, path
    assert 'def boolean' not in (root / 'mhseals_nav/startup_projection.py').read_text()
