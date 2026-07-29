#!/bin/bash
# Mission-console entrypoint: CONSOLE_PORT overrides the listen port (8080).
set -e
cd "$(dirname "$0")"
exec bin/mission_console autopilot.yaml "${CONSOLE_PORT:-8080}" web
