"""orin.launch.py: the Orin's single launch file."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.substitutions import FindPackageShare

ROOT = Path(__file__).parents[1]


def includes(monkeypatch):
    monkeypatch.setattr(FindPackageShare, 'perform', lambda self, context: str(ROOT))
    spec = importlib.util.spec_from_file_location('orin_launch', ROOT / 'launch/orin.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))
    found = {}
    for entity in module.generate_launch_description().entities:
        assert isinstance(entity, IncludeLaunchDescription)
        source = entity.launch_description_source
        source.get_launch_description(context)  # loads the real file and expands its path
        path = source.location
        found[Path(path).name] = {text(k): text(v) for k, v in entity.launch_arguments}
    return found


def test_orin_runs_camera_zed_perception_and_hardware_sensors(monkeypatch):
    assert includes(monkeypatch) == {
        'camera.launch.py': {},
        'objects.launch.py': {'backend': 'zed', 'sim': 'false'},
        'sensors.launch.py': {'sim': 'false'},
    }
