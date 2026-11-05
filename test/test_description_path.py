"""robot_geometry finds the description through layers.package_path."""
from pathlib import Path

from mhseals_nav import robot_geometry as rg


def test_the_description_is_this_source_tree_s_and_the_one_cmake_installs():
    root = Path(__file__).parents[1]
    assert rg.description_dir() == root / 'description'
    assert rg.description_path().is_file()
    assert 'description' in (root / 'CMakeLists.txt').read_text()
