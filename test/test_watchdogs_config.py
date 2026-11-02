"""config/watchdogs.yaml: one strict file, defaults < section < tune < overrides."""

import math

from mhseals_nav.yaml_params import watchdog_config
import pytest


def test_watchdogs_yaml_is_strict_and_reads_old_override_keys_for_one_release():
    warned = []
    cfg = watchdog_config('fence_watchdog', {'margin_m': 3.0, 'outward_angle_deg': 30.0,
                                             'course.corners': [1.0]}, warned.append)
    assert cfg['margin'] == 3.0 and cfg['outward_angle'] == pytest.approx(math.radians(30))
    assert len(warned) == 2 and all('deprecated' in w for w in warned)
    assert cfg['course'] == {'corners': [1.0]} and cfg['repeat'] == 1.0     # defaults apply
    assert watchdog_config('approach_guard')['rate'] == 5.0                  # its section wins
    with pytest.raises(ValueError, match='link_watchdog has no typo'):
        watchdog_config('link_watchdog', {'typo': 1})
    with pytest.raises(ValueError, match='no nothing section'):
        watchdog_config('nothing')
