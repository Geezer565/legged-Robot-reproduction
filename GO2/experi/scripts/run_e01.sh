#!/usr/bin/env bash
set -eo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
run_selected --study main --profile constant --terrain flat --trajectory velocity "$@"
