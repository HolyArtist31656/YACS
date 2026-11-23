# Downloads FRC 6328 "Darwin" (2026) MIT-licensed CAD meshes next to yacsim-robot.json
# and copies the Gradle wrapper + Commands v3 / YAMS / YACSim vendordeps from this repo.
param(
    [string]$YacsRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
)

$ErrorActionPreference = "Stop"
$projDir = $PSScriptRoot
$vendordepSrc = Join-Path $YacsRoot "vendordep"
$existingDarwinVendordeps = Join-Path $YacsRoot "examples\commands3\darwin_6328_2027\vendordeps"

Write-Host "1/3 Copying Gradle wrapper from $vendordepSrc ..."
Copy-Item "$vendordepSrc\gradlew", "$vendordepSrc\gradlew.bat", "$vendordepSrc\gradle" -Destination $projDir -Recurse -Force

Write-Host "2/3 Copying vendordeps (CommandsV3, YAMS, Phoenix 6 if present, YACSim) ..."
$vdir = Join-Path $projDir "vendordeps"
New-Item -ItemType Directory -Force -Path $vdir | Out-Null
Copy-Item "$vendordepSrc\vendordeps\CommandsV3.json", "$vendordepSrc\vendordeps\yams_commands3.json" -Destination $vdir -Force

if (Test-Path $existingDarwinVendordeps) {
    Get-ChildItem $existingDarwinVendordeps -Filter "*Phoenix*.json" | ForEach-Object {
        Copy-Item $_.FullName -Destination $vdir -Force
    }
}

$yacsimJsonPath = Join-Path $vendordepSrc "build\vendordeps\yacsim_commands3.json"
if (-not (Test-Path $yacsimJsonPath)) {
    throw "Missing $yacsimJsonPath. Run '.\gradlew publish generateVendordepJson' in $vendordepSrc first."
}
$localMavenUrl = "file:///" + ($vendordepSrc -replace "\\", "/") + "/build/maven/releases/"
$utf8NoBom = [System.Text.UTF8Encoding]::new($false)
$yacsimJson = [System.IO.File]::ReadAllText($yacsimJsonPath) -replace "https://holyartist31656\.github\.io/YACS/maven/releases/", $localMavenUrl
[System.IO.File]::WriteAllText((Join-Path $vdir "yacsim_commands3.json"), $yacsimJson, $utf8NoBom)

Write-Host "3/3 Downloading FRC 6328 Darwin CAD (.glb) next to yacsim-robot.json ..."
$rawBase = "https://raw.githubusercontent.com/Mechanical-Advantage/RobotCode2026Public/main/ascope_assets/Robot_Darwin"
$manifestDir = Join-Path $projDir "src\main\deploy\yacsim\darwin"
New-Item -ItemType Directory -Force -Path $manifestDir | Out-Null

$files = @("model.glb", "model_0.glb", "model_1.glb", "model_2.glb", "model_3.glb")
foreach ($f in $files) {
    $dst = Join-Path $manifestDir $f
    if (-not (Test-Path $dst)) {
        Write-Host "   Downloading $f ..."
        Invoke-WebRequest -Uri "$rawBase/$f" -OutFile $dst
    } else {
        Write-Host "   Keeping existing $f"
    }
}

Write-Host ""
Write-Host "Done! Launch simulation with:"
Write-Host "  .\gradlew run `"-Dorg.gradle.java.home=C:\Users\Public\wpilib\2027_alpha7\jdk`""
