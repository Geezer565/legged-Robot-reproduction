$ErrorActionPreference = "Stop"

$ConfigRoot = Split-Path -Parent $PSScriptRoot
$ComposeFile = Join-Path $ConfigRoot "compose.yaml"
$Workspace = "D:\GO2\code\legged_robot_workspace"
$DockerCli = "C:\Users\lenovo\AppData\Local\Programs\DockerDesktop\resources\bin\docker.exe"
$DockerDesktop = "C:\Users\lenovo\AppData\Local\Programs\DockerDesktop\Docker Desktop.exe"

foreach ($RequiredPath in @($ComposeFile, $Workspace, $DockerCli, $DockerDesktop)) {
    if (-not (Test-Path -LiteralPath $RequiredPath)) {
        throw "Missing required path: $RequiredPath"
    }
}

& $DockerCli info *> $null
if ($LASTEXITCODE -ne 0) {
    Start-Process -FilePath $DockerDesktop -WindowStyle Hidden
    $Ready = $false
    for ($Attempt = 0; $Attempt -lt 60; $Attempt++) {
        Start-Sleep -Seconds 2
        & $DockerCli info *> $null
        if ($LASTEXITCODE -eq 0) {
            $Ready = $true
            break
        }
    }
    if (-not $Ready) {
        throw "Docker Desktop did not become ready within 120 seconds."
    }
}

& $DockerCli compose -f $ComposeFile up -d --build
if ($LASTEXITCODE -ne 0) {
    throw "Unable to start the development container."
}

& $DockerCli compose -f $ComposeFile exec dev bash

