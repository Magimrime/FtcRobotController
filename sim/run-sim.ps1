# BIOBUZZ AUTO simulator: compiles the REAL TeamCode OpModes (unchanged) against the fake FTC SDK in
# sim/sdk plus the simulator in sim/src, then runs it. Needs only a JDK (javac/java on PATH); no robot,
# no Android Studio, no Gradle. Run from anywhere:
#   .\sim\run-sim.ps1                                   the full matrix (Auto4 + stress + Auto3)
#   .\sim\run-sim.ps1 matrix suite=quick                16 fast Auto4 matches
#   .\sim\run-sim.ps1 run alliance=blue start=far up=far noise=harsh seed=3
#   .\sim\run-sim.ps1 run opmode=Auto3 up=far
# Results: sim\results\ (summary.md + per-run .log, .csv, .svg).
param([Parameter(ValueFromRemainingArguments = $true)] [string[]] $SimArgs)
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
Push-Location $repo
try {
    $teamcode = 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode'
    $opmodes = @('Auto3', 'Auto4', 'Auto5')   # the OpModes the simulator can run (compiled straight from TeamCode)
    $sources = @(Get-ChildItem -Recurse -Filter *.java -Path sim/sdk, sim/src | ForEach-Object { $_.FullName })
    $opmodes = @($opmodes | Where-Object { Test-Path (Join-Path $teamcode "$_.java") })  # skip any not in this checkout
    $sources += $opmodes | ForEach-Object { (Resolve-Path (Join-Path $teamcode "$_.java")).Path }
    $out = 'sim/out'
    if (Test-Path $out) { Remove-Item -Recurse -Force $out }
    New-Item -ItemType Directory -Force $out | Out-Null
    $argfile = Join-Path $out 'sources.txt'
    $sources | ForEach-Object { '"' + ($_ -replace '\\', '/') + '"' } | Set-Content -Encoding ascii $argfile
    & javac -nowarn -encoding UTF-8 -d $out "@$argfile"
    if ($LASTEXITCODE -ne 0) { throw 'compile failed (see above)' }
    if (-not $SimArgs -or $SimArgs.Count -eq 0) { $SimArgs = @('matrix') }
    & java -cp $out ftcsim.Main @SimArgs
} finally {
    Pop-Location
}
