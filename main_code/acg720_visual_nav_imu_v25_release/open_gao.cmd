@echo off
setlocal
set "GAO_EXE=C:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gao_analyzer.exe"
if not exist "%GAO_EXE%" (
  echo Open debug/visual_nav_imu_v25.rao in Gowin Analyzer manually.
  pause
  exit /b 1
)
set "V25_FS=%~dp0prebuilt\25_V25_VISUAL_IMU_0x2501.fs"
if not exist "%V25_FS%" set "V25_FS=%~dp0impl\pnr\ao_0.fs"
start "" "%GAO_EXE%" -gao "%~dp0debug\visual_nav_imu_v25.rao" -series GW5AT -device GW5AT-60B -fs "%V25_FS%"
