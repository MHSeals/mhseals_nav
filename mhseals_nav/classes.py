"""The object classes, config/classes.yaml: one list for detection, tracking, fusion and tasks."""

from functools import lru_cache

from mhseals_nav.yaml_params import config_path


@lru_cache(maxsize=None)
def _load(path):
    from mhseals_nav.navigation_config import load_strict
    doc = load_strict(path.read_text(), path)
    if not isinstance(doc, dict) or set(doc) != {'default_radius', 'classes'}:
        raise ValueError(f'{path}: expected default_radius and classes')
    ids = [entry['id'] for entry in doc['classes'].values()]
    if sorted(ids) != list(range(len(ids))):
        raise ValueError(f'{path}: ids must be 0..{len(ids) - 1}, each once')
    return doc


def load(path=None):
    """{'default_radius': r, 'classes': {name: {'id', 'radius'}}}."""
    return _load(path or config_path('classes.yaml'))


def names(path=None):
    """Class names in detector id order."""
    classes = load(path)['classes']
    return sorted(classes, key=lambda name: classes[name]['id'])


def radii(path=None):
    """{name: radius, 'default': radius} as the tracker, its grid and fusion read them."""
    doc = load(path)
    return {'default': doc['default_radius'],
            **{name: entry['radius'] for name, entry in doc['classes'].items()}}


def with_radius(tracker):
    """A tracker config with ``radius`` (from classes.yaml unless it already has one)."""
    return tracker if 'radius' in tracker else {**tracker, 'radius': radii()}
