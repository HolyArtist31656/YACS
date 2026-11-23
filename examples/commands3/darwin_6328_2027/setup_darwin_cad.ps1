# Downloads FRC 6328 "Darwin" (2026) MIT-licensed CAD assets (.glb + config.json) from GitHub.
$ErrorActionPreference = "Stop"
$projDir = $PSScriptRoot
$rawBase = "https://raw.githubusercontent.com/Mechanical-Advantage/RobotCode2026Public/main/ascope_assets/Robot_Darwin"
$modelsDir = Join-Path $projDir "src\main\deploy\yacsim\models\darwin"
$ascopeDir = Join-Path $projDir "advantagescope-assets\Robot_Darwin"
New-Item -ItemType Directory -Force -Path $modelsDir, $ascopeDir | Out-Null

$files = @("config.json", "model.glb", "model_0.glb", "model_1.glb", "model_2.glb", "model_3.glb")
foreach ($f in $files) {
    $url = "$rawBase/$f"
    $dstModels = Join-Path $modelsDir $f
    $dstAscope = Join-Path $ascopeDir $f
    Write-Host "Downloading $f ..."
    Invoke-WebRequest -Uri $url -OutFile $dstModels
    Copy-Item $dstModels -Destination $dstAscope -Force
}

Write-Host ""
Write-Host "Done! Next steps:"
Write-Host "  1. Copy $ascopeDir directly into AdvantageScope's User Assets folder (Help -> Show Assets Folder)."
Write-Host "  2. Launch simulation: .\gradlew run `"-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk`""
