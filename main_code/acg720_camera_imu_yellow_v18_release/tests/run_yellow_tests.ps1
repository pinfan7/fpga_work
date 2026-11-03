param([string]$IcarusBin='D:\Gowin\tools\iverilog\mingw64\bin')
$ErrorActionPreference='Stop'
$env:PATH="D:\Git\mingw64\bin;$IcarusBin;"+$env:PATH
$taskTemp=Join-Path $PSScriptRoot 'sim_temp'
New-Item -ItemType Directory -Path $taskTemp -Force | Out-Null
$env:TEMP=$taskTemp;$env:TMP=$taskTemp;$env:TMPDIR=$taskTemp
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    foreach($testName in @('yellow_cup','yellow_overlay','yellow_fullres')) {
        & "$IcarusBin\iverilog.exe" -g2012 -Wall -s ($testName+'_tb') -o ("tests/$testName.vvp") rtl/yellow_cup_detector.v rtl/yellow_box_overlay.v ("tests/"+$testName+"_tb.sv")
        if($LASTEXITCODE -ne 0){throw "Compile failed: $testName"}
        & "$IcarusBin\vvp.exe" ("tests/$testName.vvp") 2>&1 | Tee-Object ("reports/"+$testName+"_sim.log")
        if($LASTEXITCODE -ne 0){throw "Simulation failed: $testName"}
    }
} finally {Pop-Location}
