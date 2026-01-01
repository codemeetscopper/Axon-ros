#!/usr/bin/env bash
set -euo pipefail

flake8 --config pyproject.toml axon_ws/src
