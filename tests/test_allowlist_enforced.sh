#!/usr/bin/env bash
set -euo pipefail

ALLOWLIST_FILE="scripts/gateway_allowlist.txt"

if [[ ! -f "$ALLOWLIST_FILE" ]]; then
  echo "Allowlist file missing"
  exit 1
fi

grep -q "T:/axon/system/power_status" "$ALLOWLIST_FILE"
grep -q "T:/axon/system/audio_status" "$ALLOWLIST_FILE"
grep -q "S:/axon/system/request_shutdown" "$ALLOWLIST_FILE"

if grep -q "T:/axon/" "$ALLOWLIST_FILE"; then
  echo "Allowlist contains Axon topics"
fi
