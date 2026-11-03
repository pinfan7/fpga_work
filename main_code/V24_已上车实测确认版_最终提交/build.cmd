@echo off
setlocal
cd /d "%~dp0"
"C:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe" build.tcl
if errorlevel 1 exit /b 1
echo Check impl\pnr\ao_0.fs and reports before programming.
pause
