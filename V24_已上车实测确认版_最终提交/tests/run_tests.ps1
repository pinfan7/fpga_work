param([string]$IcarusBin='D:/Gowin/tools/iverilog/mingw64/bin')
$ErrorActionPreference='Stop'
$env:PATH="D:/Git/mingw64/bin;$IcarusBin;"+$env:PATH
$taskSimTemp=Join-Path $PSScriptRoot 'sim_temp'
New-Item -ItemType Directory -Path $taskSimTemp -Force | Out-Null
$env:TEMP=$taskSimTemp;$env:TMP=$taskSimTemp;$env:TMPDIR=$taskSimTemp
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
 $tof=@('rtl/tof/uart_rx.v','rtl/tof/uart_tx.v','rtl/tof/tof_channel.v')
 $drive=@('rtl/drive/ackermann_auto_top.v','rtl/drive/nav_controller.v','rtl/drive/nav_pose.v',
          'rtl/drive/car_closed_loop.v','rtl/drive/button_event.v','rtl/drive/servo_pwm.v')
 $cases=[ordered]@{
  pose=@('rtl/drive/nav_pose.v')
  imu_heading=@('rtl/imu_attitude.v','rtl/cordic_atan2.v')
  navigation=@('rtl/drive/nav_controller.v','rtl/drive/nav_pose.v')
  nav_limits=@('rtl/drive/nav_controller.v')
  top_route=$drive+$tof+@('rtl/tof/tof3_subsystem.v','tests/vision_boundary_model.sv',
                        'rtl/imu_session_guard.v','rtl/yellow_cup_detector.v')
  motor_chain=@('rtl/drive/car_closed_loop.v')
  encoder=@('rtl/drive/car_closed_loop.v')
  tof3=$tof+@('tests/tof_reference_top.v')
  default_uart=$tof
 }
 foreach($testName in $cases.Keys) {
  $argsCompile=@('-g2012','-Wall','-s',($testName+'_tb'),'-o',("tests/$testName.vvp"))+$cases[$testName]+@("tests/${testName}_tb.sv")
  & "$IcarusBin/iverilog.exe" @argsCompile
  if($LASTEXITCODE -ne 0){throw "Compile failed: $testName"}
  & "$IcarusBin/vvp.exe" "tests/$testName.vvp" 2>&1 | Tee-Object "reports/${testName}_sim.log"
  if($LASTEXITCODE -ne 0){throw "Simulation failed: $testName"}
  if(!(Select-String -LiteralPath "reports/${testName}_sim.log" -SimpleMatch 'PASS:' -Quiet)){throw "No completion: $testName"}
 }
} finally {Pop-Location}
