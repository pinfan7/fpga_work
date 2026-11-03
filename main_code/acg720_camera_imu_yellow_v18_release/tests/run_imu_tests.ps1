$ErrorActionPreference='Stop'
$bin='D:/Gowin/tools/iverilog/mingw64/bin'
$env:PATH="D:/Git/mingw64/bin;$bin;"+$env:PATH
$taskTemp=Join-Path $PSScriptRoot 'sim_temp'
New-Item -ItemType Directory -Path $taskTemp -Force | Out-Null
$env:TEMP=$taskTemp;$env:TMP=$taskTemp;$env:TMPDIR=$taskTemp
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    & "$bin/iverilog.exe" -g2012 -Wall -s imu_i2c_tb -o tests/imu_i2c.vvp rtl/imu_recovery_i2c_master.v tests/imu_i2c_tb.sv
    if($LASTEXITCODE -ne 0){throw 'Compile failed'}
    & "$bin/vvp.exe" tests/imu_i2c.vvp 2>&1 | Tee-Object reports/imu_i2c_sim.log
    if($LASTEXITCODE -ne 0){throw 'Simulation failed'}
    & "$bin/iverilog.exe" -g2012 -Wall -s imu_acquire_tb -o tests/imu_acquire.vvp rtl/icm42688_acquire.v rtl/imu_session_guard.v tests/imu_acquire_tb.sv
    if($LASTEXITCODE -ne 0){throw 'Acquisition compile failed'}
    & "$bin/vvp.exe" tests/imu_acquire.vvp 2>&1 | Tee-Object reports/imu_acquire_sim.log
    if($LASTEXITCODE -ne 0){throw 'Acquisition simulation failed'}
} finally {Pop-Location}
