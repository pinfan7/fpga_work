$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot
$bundled = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
if (Test-Path $bundled) { $python = $bundled }
elseif (Get-Command python -ErrorAction SilentlyContinue) { $python = (Get-Command python).Source }
else { throw 'Please install Python 3.10 or newer, then run again.' }
& $python -c 'import importlib.util,sys; sys.exit(0 if importlib.util.find_spec("PIL") else 1)'
if ($LASTEXITCODE -ne 0) {
    & $python -m pip install -r requirements.txt
    if ($LASTEXITCODE -ne 0) { throw 'Pillow installation failed.' }
}
& $python server.py
Read-Host 'Press Enter to close'
