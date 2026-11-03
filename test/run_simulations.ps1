param(
    [string]$IcarusBin = $env:IVERILOG_BIN,
    [ValidateSet('all','v6','servo','tof')][string]$Project = 'all'
)
$ErrorActionPreference = 'Stop'
if ($IcarusBin) {
    $compiler = Join-Path $IcarusBin 'iverilog.exe'
    $runtime = Join-Path $IcarusBin 'vvp.exe'
    $env:PATH = "$IcarusBin;" + $env:PATH
} else {
    $compiler = (Get-Command iverilog -ErrorAction Stop).Source
    $runtime = (Get-Command vvp -ErrorAction Stop).Source
}
$projects = @(
    @{ Key='v6'; Folder='acg720_ackermann_auto_v6_gao_release'; Tests=@('noimu_sequence_tb','noimu_top_tb') },
    @{ Key='servo'; Folder='acg720_ackermann_servo_v2'; Tests=@('servo_pwm_tb','servo_top_tb') },
    @{ Key='tof'; Folder='acg720_tof3_test'; Tests=@('tof3_tb','default_uart_tb') }
)
$taskTemp = Join-Path ([System.IO.Path]::GetTempPath()) ('fpga-sim-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $taskTemp | Out-Null
try {
    foreach ($entry in $projects) {
        if ($Project -ne 'all' -and $Project -ne $entry.Key) { continue }
        $root = Join-Path $PSScriptRoot $entry.Folder
        $sources = @(Get-ChildItem -LiteralPath (Join-Path $root 'rtl') -Filter '*.v' | ForEach-Object FullName)
        $reports = Join-Path $root 'reports'
        New-Item -ItemType Directory -Force -Path $reports | Out-Null
        foreach ($name in $entry.Tests) {
            $output = Join-Path $taskTemp ($name + '.vvp')
            & $compiler -g2012 -Wall -s $name -o $output @sources (Join-Path $root "sim/$name.sv")
            if ($LASTEXITCODE -ne 0) { throw "Compile failed: $name" }
            $result = & $runtime $output 2>&1
            $status = $LASTEXITCODE
            $result | Tee-Object -FilePath (Join-Path $reports ($name + '.log'))
            if ($status -ne 0 -or -not ($result -match '^PASS')) { throw "Simulation failed: $name" }
        }
    }
} finally {
    # Only the unique directory created by this invocation is removed.
    Remove-Item -LiteralPath $taskTemp -Recurse -Force
}
