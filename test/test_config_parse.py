"""Every YAML the package ships parses (a broken help.yaml once blocked a landing)."""

from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
FILES = sorted(ROOT.glob('config/**/*.yaml')) + [ROOT / 'help.yaml']


@pytest.mark.parametrize('path', FILES, ids=lambda p: str(p.relative_to(ROOT)))
def test_parses(path):
    assert yaml.safe_load(path.read_text()) is not None


def test_every_node_config_has_ros_parameters():
    """A node config with a top-level '/**' holds its parameters under ros__parameters."""
    for path in FILES:
        doc = yaml.safe_load(path.read_text())
        if isinstance(doc, dict) and '/**' in doc:
            assert isinstance(doc['/**'].get('ros__parameters'), dict), path
