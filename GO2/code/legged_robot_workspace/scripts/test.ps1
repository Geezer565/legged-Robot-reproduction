$ErrorActionPreference = "Stop"
$Workspace = Split-Path -Parent $PSScriptRoot
docker compose -f (Join-Path $Workspace "compose.yaml") exec dev bash /workspace/scripts/test.sh
