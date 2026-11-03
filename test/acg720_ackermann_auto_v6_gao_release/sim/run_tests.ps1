param([string]$IcarusBin = $env:IVERILOG_BIN)
& (Join-Path $PSScriptRoot '../../run_simulations.ps1') -Project v6 -IcarusBin $IcarusBin
