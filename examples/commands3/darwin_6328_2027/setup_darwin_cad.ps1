# Downloads FRC 6328 "Darwin" (2026) MIT-licensed CAD assets from GitHub
# and copies your local Gradle wrapper + Commands v3 / YAMS / YACSim vendordeps.
param(
    [string]$YacsRoot = "C:\Users\nmoe1\Documents\GitHub\YACS"
)

$ErrorActionPreference = "Stop"
$projDir = $PSScriptRoot
$vendordepSrc = Join-Path $YacsRoot "vendordep"

Write-Host "1/3 Copying Gradle wrapper from $vendordepSrc ..."
Copy-Item "$vendordepSrc\gradlew", "$vendordepSrc\gradlew.bat", "$vendordepSrc\gradle" -Destination $projDir -Recurse -Force

Write-Host "2/3 Copying vendordeps (CommandsV3, YAMS, YACSim) ..."
$vdir = Join-Path $projDir "vendordeps"
New-Item -ItemType Directory -Force -Path $vdir | Out-Null
Copy-Item "$vendordepSrc\vendordeps\CommandsV3.json", "$vendordepSrc\vendordeps\yams_commands3.json" -Destination $vdir -Force

$yacsimJsonPath = Join-Path $vendordepSrc "build\vendordeps\yacsim_commands3.json"
if (-not (Test-Path $yacsimJsonPath)) {
    throw "Missing $yacsimJsonPath. Run '.\gradlew publish generateVendordepJson' in $vendordepSrc first."
}
$localMavenUrl = "file:///" + ($vendordepSrc -replace "\\", "/") + "/build/maven/releases/"
$yacsimJson = (Get-Content $yacsimJsonPath -Raw) -replace "https://cdn\.yassrobotics\.com/", $localMavenUrl
Set-Content -Path (Join-Path $vdir "yacsim_commands3.json") -Value $yacsimJson -Encoding UTF8

Write-Host "3/3 Downloading FRC 6328 Darwin CAD (.glb + config.json) from GitHub ..."
$rawBase = "https://raw.githubusercontent.com/Mechanical-Advantage/RobotCode2026Public/main/ascope_assets/Robot_Darwin"
$modelsDir = Join-Path $projDir "src\main\deploy\yacsim\models\darwin"
$ascopeDir = Join-Path $projDir "advantagescope-assets\Robot_Darwin"
New-Item -ItemType Directory -Force -Path $modelsDir, $ascopeDir | Out-Null

$files = @("config.json", "model.glb", "model_0.glb", "model_1.glb", "model_2.glb", "model_3.glb")
foreach ($f in $files) {
    $url = "$rawBase/$f"
    $dstAscope = Join-Path $ascopeDir $f
    Write-Host "   Downloading $f ..."
    Invoke-WebRequest -Uri $url -OutFile $dstAscope
    if ($f.EndsWith(".glb")) {
        Copy-Item $dstAscope -Destination (Join-Path $modelsDir $f) -Force
    }
}

Write-Host ""
Write-Host "Done! Next steps:"
Write-Host "  1. Install CTRE Phoenix 6 (2027 alpha) vendordep into $vdir via VS Code (WPILib: Manage Vendor Libraries -> Install online)."
Write-Host "  2. Copy $ascopeDir directly into AdvantageScope's User Assets folder (Help -> Show Assets Folder)."
Write-Host "  3. Launch simulation: .\gradlew run `"-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk`""
