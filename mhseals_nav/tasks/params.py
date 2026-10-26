"""Task and mission parameters, free of py_trees and ROS: the run budget and the profile.

The mission host builds its trees from these; a harness that only needs the run budget
(``load_params('mission', path)['timeout']``) or a task's settings imports this alone.
"""

from pathlib import Path

from mhseals_nav.navigation_config import load_strict


def config_path(name):
    """Return a task YAML's path in the installed share, else in this source tree."""
    path = None
    try:
        from ament_index_python.packages import (get_package_share_directory,
                                                 PackageNotFoundError)
    except ImportError:  # no ROS on this host: the source tree only
        pass
    else:
        try:
            path = Path(get_package_share_directory('mhseals_nav')) / 'config/tasks' / name
        except PackageNotFoundError:
            pass
    if path is None or not path.exists():
        path = Path(__file__).resolve().parents[2] / 'config/tasks' / name
    return str(path)


def load_params(task, path=None, profile=None):
    """Load a task's YAML; for the mission, each task's own YAML under its name.

    A mission then takes its ``profile`` (the argument, else the YAML's own key): the
    overlay config/tasks/profiles/<profile>.yaml, merged key by key, may only change
    keys the mission already has. A ``timeslot`` {slot_s, setup_reserve_s} sets the
    run's ``timeout`` (the run budget): what the slot leaves once set-up is kept back.
    """
    path = path or config_path(f'{task}.yaml')
    params = load_strict(Path(path).read_text(encoding='utf-8'), path) or {}
    if task == 'mission':
        for name in params['tasks']:
            params[name] = {**load_params(name), **params.get(name, {})}
        profile = profile or params.get('profile')
        if profile:
            source = config_path(f'profiles/{profile}.yaml')
            top = load_strict(Path(source).read_text(encoding='utf-8'), source) or {}
            params = overlay(params, top, source)
            for name in params['tasks']:  # a profile's task list may name a new one
                params.setdefault(name, load_params(name))
            params['profile'] = profile
        params = timeslot(params)
    return params


def overlay(base, top, source, at=''):
    """Return ``base`` with ``top`` merged in key by key; a key ``base`` lacks is an error."""
    merged = dict(base)
    for key, value in top.items():
        if key not in base:
            raise KeyError(f'{source}: {at}{key} is not a mission key')
        merged[key] = (overlay(base[key], value, source, f'{at}{key}.')
                       if isinstance(value, dict) and isinstance(base[key], dict) else value)
    return merged


def timeslot(params):
    """Set the mission's ``timeout`` from its ``timeslot``: slot_s less setup_reserve_s.

    The slot is the competition's (900 s); set-up, launch and recovery come off it first.
    The exit reserve, each task's admission and its own limit all count from this budget.
    """
    slot = params.get('timeslot')
    if not slot:
        return params
    if 'timeout' in params:
        raise KeyError('a mission has a timeslot or a timeout, not both')
    budget = float(slot['slot_s']) - float(slot['setup_reserve_s'])
    if budget <= float(params.get('exit_reserve', 0.0)):
        raise ValueError(f'timeslot leaves {budget:.0f} s: no more than the exit reserve')
    return {**params, 'timeout': budget}
