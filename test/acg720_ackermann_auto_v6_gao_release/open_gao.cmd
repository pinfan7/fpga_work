@echo off
setlocal
set "GAO_EXE=C:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gao_analyzer.exe"
if not exist "%GAO_EXE%" (
  echo Gowin analyzer not found. Open debug/drive_v6.rao in your Gowin analyzer.
  pause
  exit /b 1
)
start "" "%GAO_EXE%" -gao "%~dp0prebuilt\drive_v6.rao" -series GW5AT -device GW5AT-60B -fs "%~dp0prebuilt\ACKERMANN_AUTO_V6_GAO_NOIMU_D16.fs"
