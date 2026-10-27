"""The run's time budget: which tasks still fit, by points per second, before the exit.

Tasks run in the mission's order, but for its ``agenda``: those go in the order the
course-layout belief plans (tasks/layout.py), which also replaces ``fits``'s verdict for
them with its own (the exit's reserve from where each ends). Before each optional task
starts, ``fits`` asks whether it is among the tasks worth the time left: the exit's
``reserve`` comes off first, then the remaining tasks are kept in order of points per
expected second until their expected times fill what is left. A task whose capability
is off (a delivery actuator not built yet) is worth no points and never fits.
"""


def value(task, capabilities):
    """Return the task's points, 0 when it ``needs`` capabilities and none of them is on."""
    needs = task.get('needs') or ()
    if needs and not any(capabilities.get(need, False) for need in needs):
        return 0.0
    return float(task.get('points', 0.0))


def choose(left_s, tasks, capabilities):
    """Return the names of ``tasks`` (dicts: name, points, expected_s) that fit ``left_s``.

    Greedy by points per expected second, the tie to the earlier task; a task worth
    nothing is never chosen.
    """
    ranked = sorted(enumerate(tasks), key=lambda it: (
        -value(it[1], capabilities) / max(it[1].get('expected_s', 1.0), 1.0), it[0]))
    kept, used = set(), 0.0
    for _, task in ranked:
        cost = float(task.get('expected_s', 0.0))
        if value(task, capabilities) > 0 and used + cost <= left_s:
            kept.add(task['name'])
            used += cost
    return kept


def fits(name, left_s, tasks, capabilities):
    """Tell whether ``name`` (the next of ``tasks``) is chosen; with the reason as text."""
    task = next(t for t in tasks if t['name'] == name)
    if value(task, capabilities) <= 0:
        if task.get('needs'):
            return False, f'{name}: capability off ({", ".join(task["needs"])})'
        return False, f'{name}: no points'
    if name in choose(left_s, tasks, capabilities):
        return True, f'{name} fits: {left_s:.0f} s left'
    return False, f'no time for {name}: {left_s:.0f} s left'
