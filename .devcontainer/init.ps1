$ErrorActionPreference = "Stop"

$root   = Split-Path -Parent $MyInvocation.MyCommand.Path
$source = Join-Path $root "docker-compose.windows.yml"
$target = Join-Path $root "docker-compose.override.yml"

Write-Host "[init] Detected host OS: windows (PowerShell)"
Copy-Item -Path $source -Destination $target -Force
Write-Host "[init] Wrote $target"
