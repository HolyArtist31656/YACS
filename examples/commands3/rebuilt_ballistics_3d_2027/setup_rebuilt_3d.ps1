# Copies Gradle wrapper + WPILib 2027 vendordeps (CommandsV3, YAMS, YACSim v2027.5.0-alpha1)
param(
    [string]$YacsRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
)
$ErrorActionPreference = "Stop"
$projDir = $PSScriptRoot
$vendordepSrc = Join-Path $YacsRoot "vendordep"
$vdir = Join-Path $projDir "vendordeps"
New-Item -ItemType Directory -Force -Path $vdir | Out-Null

Copy-Item "$vendordepSrc\gradlew", "$vendordepSrc\gradlew.bat", "$vendordepSrc\gradle" -Destination $projDir -Recurse -Force
Copy-Item "$vendordepSrc\vendordeps\CommandsV3.json", "$vendordepSrc\vendordeps\yams_commands3.json" -Destination $vdir -Force
Copy-Item "$vendordepSrc\build\vendordeps\yacsim_commands3.json" -Destination (Join-Path $vdir "yacsim_commands3.json") -Force
Write-Host "Setup complete for rebuilt_ballistics_3d_2027!"
