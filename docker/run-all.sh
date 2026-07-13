#!/bin/bash
# All-in-one entry point for the autopilot application image: runs the
# autopilot and the mission-control web console together, exits when either
# dies, and forwards SIGTERM/SIGINT so `docker stop` shuts both down.
#
# CONSOLE_PORT overrides the web console port (default 8080).

set -u
cd "$(dirname "$0")"

bin/autopilot autopilot.yaml &
AUTOPILOT_PID=$!

bin/mission_console autopilot.yaml "${CONSOLE_PORT:-8080}" web &
CONSOLE_PID=$!

shutdown() {
  kill -TERM "${AUTOPILOT_PID}" "${CONSOLE_PID}" 2>/dev/null
}
trap shutdown TERM INT

wait -n "${AUTOPILOT_PID}" "${CONSOLE_PID}"
STATUS=$?
shutdown
wait 2>/dev/null
exit "${STATUS}"
