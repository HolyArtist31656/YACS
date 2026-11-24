# Sets up Gradle wrapper + 2027 vendordeps (CommandsV3, YAMS, YACSim, REVLib 2027)
# for FRC Team 2181 GEARS (gears_2181_2027).
param(
    [string]$YacsRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
)

$ErrorActionPreference = "Stop"
$projDir = $PSScriptRoot
$vendordepSrc = Join-Path $YacsRoot "vendordep"
$vdir = Join-Path $projDir "vendordeps"
New-Item -ItemType Directory -Force -Path $vdir | Out-Null

Write-Host "1/3 Copying Gradle wrapper from $vendordepSrc ..."
Copy-Item "$vendordepSrc\gradlew", "$vendordepSrc\gradlew.bat", "$vendordepSrc\gradle" -Destination $projDir -Recurse -Force

Write-Host "2/3 Copying CommandsV3, YAMS, and YACSim vendordeps ..."
Copy-Item "$vendordepSrc\vendordeps\CommandsV3.json", "$vendordepSrc\vendordeps\yams_commands3.json" -Destination $vdir -Force
$pubYacsim = Join-Path $YacsRoot "examples\commands3\arm_and_elevator_2027\vendordeps\yacsim_commands3.json"
Copy-Item $pubYacsim (Join-Path $vdir "yacsim_commands3.json") -Force

Write-Host "3/3 Downloading REVLib 2027 vendordep JSON ..."
Invoke-WebRequest -Uri "https://software-metadata.revrobotics.com/REVLib-2027.json" -OutFile (Join-Path $vdir "REVLib-2027.json")

Write-Host ""
Write-Host "Done! Launch simulation with:"
Write-Host "  .\gradlew run --refresh-dependencies `"-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk`""
