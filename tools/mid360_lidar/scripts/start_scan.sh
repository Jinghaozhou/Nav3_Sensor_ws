#!/usr/bin/env bash
set -e
SCAN_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
SENSOR_WS="$(cd "$SCAN_ROOT/../.." && pwd -P)"
LOCALIZATION_WS="${NAV3_LOCALIZATION_WS:-$(dirname "$SENSOR_WS")/Nav3_Localization_ws}"
source /opt/ros/humble/setup.bash
if [[ -f "$SENSOR_WS/../../install/setup.bash" && "$(basename "$(dirname "$SENSOR_WS")")" == modules ]]; then
  source "$SENSOR_WS/../../install/setup.bash"
else
  source "$SENSOR_WS/install/setup.bash"
  source "$LOCALIZATION_WS/install/setup.bash"
fi
exec /usr/bin/python3 "$SCAN_ROOT/scripts/start_scan.py" "$@"
