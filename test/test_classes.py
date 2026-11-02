"""config/classes.yaml is the one class list: the detector, the tracker and the tasks agree."""

from fnmatch import fnmatch
from pathlib import Path
import re

from mhseals_nav import classes
import yaml

ROOT = Path(__file__).parents[1]
# In the 2026 rules and the tasks, not yet a class the detector reports.
NOT_DETECTED = {'orange_target_boat'}


def test_the_detector_runs_the_class_list_in_id_order():
    detection = yaml.safe_load((ROOT / 'config/custom_object_detection.yaml').read_text())[
        '/**']['ros__parameters']['object_detection']
    rows = sorted((v['model_class_id'], v['label']) for k, v in detection.items()
                  if k.startswith('class_'))
    assert [label for _, label in rows] == classes.names()
    assert [i for i, _ in rows] == list(range(detection['custom_class_count']))


def test_every_class_has_a_tracker_group_and_a_radius():
    tracker = yaml.safe_load((ROOT / 'config/tracker.yaml').read_text())['/**']['ros__parameters']
    for name in classes.names():
        assert any(fnmatch(name, p) for group, spec in tracker['class_groups'].items()
                   if group != 'unknown' for p in spec['match']), name
    radii = classes.radii()
    assert set(radii) == {'default', *classes.names()} and all(r > 0 for r in radii.values())
    assert 'radius' not in tracker                       # classes.yaml holds it
    assert classes.with_radius(tracker)['radius'] == radii


def test_tasks_name_only_listed_classes():
    pattern = re.compile(r'\b[a-z]+_(?:buoy|target_boat)\b|\bblack_(?:cross|triangle)\b')
    used = {m for path in (ROOT / 'config/tasks').rglob('*.yaml')
            for m in pattern.findall(path.read_text())}
    assert used - NOT_DETECTED <= set(classes.names())
