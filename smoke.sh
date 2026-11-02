#!/usr/bin/env bash
# Smoke tier (< 60 s, gates landing): config parse and the pure-python units, no ROS needed.
# Tests named test_node_* start ROS nodes and are left to the full suite (CI / nightly).
# Usage: ./smoke.sh [--impact] [extra pytest args]
#   --impact: only the smoke tests mhseals_agents' timpact picks against origin/main (ALL, or no
#   timpact: the full smoke). A plain run goes through its tcache (tier smoke) when that exists,
#   so a tree that passed is not run again. mhseals_agents is beside this clone, or MHSEALS_AGENTS.
set -euo pipefail
cd "$(dirname "$0")"
SMOKE=(test/test_config_parse.py test/test_help_yaml.py test/test_config_schema.py
    test/test_sources_compile.py test/test_strict_load_errors.py test/test_launch_boolean.py
    test/test_link_watchdog.py test/test_fence_watchdog.py test/test_hold_watchdog.py
    test/test_hold_release.py test/test_costmap_watchdog.py test/test_ekf_input_watchdog.py
    test/test_fcu_watchdog.py test/test_approach_guard.py test/test_navigation_config.py
    test/test_track_filter.py test/test_track_grid.py test/test_course_map.py
    test/test_nearby.py)
RUN=(python3 -m pytest -q -p no:cacheprovider -m 'not linter' -k 'not test_node')
clone=$(git rev-parse --path-format=absolute --git-common-dir 2>/dev/null || echo "$PWD/.git")
AGENTS=${MHSEALS_AGENTS:-$(dirname "$(dirname "$clone")")/mhseals_agents}
# its parent on the path turns on the quarantine plugin (the root conftest.py); this checkout still comes first
[[ $(basename "$AGENTS") == mhseals_agents && -d $AGENTS ]] && export PYTHONPATH=${PYTHONPATH:+$PYTHONPATH:}$(dirname "$AGENTS")

if [[ ${1:-} == --impact ]]; then
    shift
    if [[ -x $AGENTS/tools/timpact ]] && picked=$("$AGENTS/tools/timpact" --repo "$PWD") &&
            ! grep -qx ALL <<<"$picked"; then
        tests=()
        for t in $picked; do
            for s in "${SMOKE[@]}"; do [[ ${t#"$PWD"/} == "$s" ]] && tests+=("$s"); done
        done
        if (( ${#tests[@]} == 0 )); then echo 'smoke --impact: no smoke test affected'; exit 0; fi
        exec "${RUN[@]}" "${tests[@]}" "$@"
    fi
    echo 'smoke --impact: full smoke' >&2
fi
if (( $# == 0 )) && [[ -x $AGENTS/tools/tcache ]]; then
    exec "$AGENTS/tools/tcache" run --repo "$PWD" --tier smoke -- "${RUN[@]}" "${SMOKE[@]}"
fi
exec "${RUN[@]}" "${SMOKE[@]}" "$@"
