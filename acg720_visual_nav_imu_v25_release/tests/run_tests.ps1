param([string]$IcarusBin='D:/Gowin/tools/iverilog/mingw64/bin',
  [string[]]$Only=@('motor_chain','encoder','balance','tof3','default_uart','nav_cdc','visual_nav','nav_integration','visual_imu','imu_heading','yellow_detect'))
$ErrorActionPreference='Stop'
$env:PATH="D:/Git/mingw64/bin;$IcarusBin;"+$env:PATH
$taskSimTemp=Join-Path $PSScriptRoot 'sim_temp'
New-Item -ItemType Directory -Path $taskSimTemp -Force | Out-Null
$env:TEMP=$taskSimTemp;$env:TMP=$taskSimTemp;$env:TMPDIR=$taskSimTemp
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
  $tof=@('rtl/tof/uart_rx.v','rtl/tof/uart_tx.v','rtl/tof/tof_channel.v')
  $drive=@('rtl/drive/ackermann_auto_top.v','rtl/drive/auto_sequence.v',
    'rtl/drive/car_closed_loop.v','rtl/drive/button_event.v','rtl/drive/servo_pwm.v')
  $cases=@{
    nav_cdc=@('rtl/nav/vision_target_cdc.v','rtl/nav/camera_frame_watchdog.v')
    visual_nav=@('rtl/nav/visual_approach_controller.v','rtl/nav/visual_imu_heading.v')
    visual_imu=@('rtl/nav/visual_imu_heading.v')
    imu_heading=@('rtl/imu_attitude.v','rtl/cordic_atan2.v')
    yellow_detect=@('rtl/yellow_cup_detector.v')
    nav_integration=$drive+$tof+@('rtl/tof/tof3_subsystem.v','rtl/nav/visual_approach_controller.v',
      'rtl/nav/visual_imu_heading.v','tests/vision_boundary_model.sv','rtl/yellow_cup_detector.v','rtl/imu_session_guard.v')
    motor_chain=@('rtl/drive/car_closed_loop.v')
    encoder=@('rtl/drive/car_closed_loop.v')
    balance=@('rtl/drive/auto_sequence.v')
    tof3=$tof+@('tests/tof_reference_top.v')
    default_uart=$tof
  }
  foreach($testName in $Only) {
    $argsCompile=@('-g2012','-s',($testName+'_tb'),'-o',("tests/$testName.vvp"))+
      $cases[$testName]+@("tests/${testName}_tb.sv")
    & "$IcarusBin/iverilog.exe" @argsCompile
    if($LASTEXITCODE -ne 0){throw "Compile failed: $testName"}
    $taskSimOutput=& "$IcarusBin/vvp.exe" "tests/$testName.vvp" 2>&1 | Tee-Object "reports/${testName}_sim.log"
    $taskSimOutput | Write-Output
    if($LASTEXITCODE -ne 0 -or !($taskSimOutput -match "^PASS") -or ($taskSimOutput -match "FATAL|ERROR")){throw "Simulation failed: $testName"}
  }
} finally {Pop-Location}
