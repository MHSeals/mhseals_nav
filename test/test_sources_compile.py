"""Every module must compile so no commit breaks git bisect."""
from pathlib import Path
import py_compile

import pytest

ROOT = Path(__file__).resolve().parent.parent
SOURCES = sorted(p for d in ('mhseals_nav', 'launch', 'scripts', 'test')
                 for p in (ROOT / d).rglob('*.py'))


@pytest.mark.parametrize('path', SOURCES, ids=lambda p: str(p.relative_to(ROOT)))
def test_source_compiles(path, tmp_path):
    py_compile.compile(str(path), cfile=str(tmp_path / 'out.pyc'), doraise=True)
