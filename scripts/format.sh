#!/usr/bin/env bash
set -euo pipefail

black axon_ws/src
isort axon_ws/src
