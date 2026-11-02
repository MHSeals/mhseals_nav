#!/usr/bin/env bash
# Smoke tier (< 60 s, gates landing): config parse and the pure-python units, no ROS needed.
# Tests named test_node_* start ROS nodes and are left to the full suite (CI / nightly).
# Usage: ./smoke.sh [extra pytest args]
set -euo pipefail
cd "$(dirname "$0")"
exec python3 -m pytest -q -p no:cacheprovider -m 'not linter' -k 'not test_node' \
    test/test_config_parse.py test/test_help_yaml.py test/test_config_schema.py \
    test/test_sources_compile.py test/test_strict_load_errors.py test/test_launch_boolean.py \
    test/test_link_watchdog.py test/test_fence_watchdog.py test/test_hold_watchdog.py \
    test/test_hold_release.py test/test_costmap_watchdog.py test/test_ekf_input_watchdog.py \
    test/test_fcu_watchdog.py test/test_approach_guard.py test/test_navigation_config.py \
    test/test_track_filter.py test/test_track_grid.py test/test_course_map.py "$@"
