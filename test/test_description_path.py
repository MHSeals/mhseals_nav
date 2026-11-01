"""robot_geometry finds the description through the installed package, else the source tree."""
from pathlib import Path
import sys
import types

from mhseals_nav import robot_geometry as rg


def fake_ament(monkeypatch, share=None):
    def get_package_share_directory(name):
        if share is None:
            raise LookupError(name)
        return str(share)
    packages = types.ModuleType('ament_index_python.packages')
    packages.get_package_share_directory = get_package_share_directory
    parent = types.ModuleType('ament_index_python')
    parent.packages = packages
    monkeypatch.setitem(sys.modules, 'ament_index_python', parent)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)


def test_an_installed_package_is_read_from_its_share_directory(monkeypatch, tmp_path):
    (tmp_path / 'description' / 'blastoise').mkdir(parents=True)
    fake_ament(monkeypatch, tmp_path)
    assert rg.description_dir() == tmp_path / 'description'
    assert rg.description_path() == tmp_path / 'description/blastoise/blastoise.xacro'


def test_an_uninstalled_package_falls_back_to_the_source_tree(monkeypatch):
    fake_ament(monkeypatch)
    assert rg.description_dir() == rg.SOURCE_DESCRIPTION
    assert rg.description_path().is_file()


def test_a_share_directory_without_the_description_falls_back(monkeypatch, tmp_path):
    fake_ament(monkeypatch, tmp_path)
    assert rg.description_dir() == rg.SOURCE_DESCRIPTION


def test_the_source_description_is_the_one_cmake_installs():
    assert rg.SOURCE_DESCRIPTION == Path(__file__).parents[1] / 'description'
    assert 'description' in (Path(__file__).parents[1] / 'CMakeLists.txt').read_text()
