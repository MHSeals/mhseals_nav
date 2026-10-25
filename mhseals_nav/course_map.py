"""The perceived course map: static tracks saved as lat/lon JSON (perception-v2 §5.6).

``$MHSEALS_DATA/course_map/<name>.json``; the tracker is its only writer. Lat/lon
because navsat_transform picks a new ``map`` origin every run.
"""

from datetime import datetime, timedelta, timezone
import json
import os
from pathlib import Path

SCHEMA = 'mhseals.course_map/1'


def data_root():
    """``$MHSEALS_DATA``, else ``$ROS_WS/bags``, else ``~/astro_dock/bags`` (data_layout)."""
    if os.environ.get('MHSEALS_DATA'):
        return Path(os.environ['MHSEALS_DATA'])
    if os.environ.get('ROS_WS'):
        return Path(os.environ['ROS_WS']) / 'bags'
    return Path.home() / 'astro_dock' / 'bags'


def path(name, root=None):
    return Path(root or data_root()) / 'course_map' / f'{name}.json'


def name_for(cfg, sim, now, pid):
    """Return the course map name: ``cfg['name']`` on the boat, a per-run file in sim.

    A sim run gets a file of its own under ``sim_dir``, so concurrent or successive sims
    (other seeds, other courses) never share priors.
    """
    if not sim:
        return cfg['name']
    return f"{cfg['sim_dir']}/{now.astimezone(timezone.utc):%Y%m%dT%H%M%SZ}_{pid}"


def iso(when):
    return when.astimezone(timezone.utc).isoformat(timespec='seconds')


def save(file, name, entries, saved):
    """Write atomically; entries are dicts of class_id, lat, lon, sigma_m, hits, last_seen."""
    file = Path(file)
    file.parent.mkdir(parents=True, exist_ok=True)
    body = {'schema': SCHEMA, 'name': name, 'saved': iso(saved), 'tracks': entries}
    temp = file.with_name(file.name + '.tmp')
    temp.write_text(json.dumps(body, indent=1))
    os.replace(temp, file)


def load(file, now, max_age_days):
    """Entries no older than ``max_age_days``; a missing or malformed file reads as []."""
    try:
        body = json.loads(Path(file).read_text())
        if body.get('schema') != SCHEMA:
            return []
        out = []
        for row in body['tracks']:
            seen = datetime.fromisoformat(row['last_seen'])
            if now - seen <= timedelta(days=max_age_days):
                out.append({'class_id': str(row['class_id']), 'lat': float(row['lat']),
                            'lon': float(row['lon']), 'sigma_m': float(row['sigma_m']),
                            'hits': int(row['hits']), 'last_seen': row['last_seen']})
        return out
    except (OSError, ValueError, KeyError, TypeError):
        return []
