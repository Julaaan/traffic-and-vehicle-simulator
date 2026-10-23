[CmdletBinding(PositionalBinding = $false)]
param(
    [switch]$Headless,
    [ValidateSet('Release', 'Debug')][string]$Configuration = 'Release',
    [Parameter(ValueFromRemainingArguments = $true)][string[]]$SimulatorArguments
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$preset = $Configuration.ToLowerInvariant()
$program = if ($Headless) { 'sim_cli.exe' } else { 'sim_viewer.exe' }
$executable = Join-Path $projectRoot "build\$preset\bin\$program"
if (-not (Test-Path -LiteralPath $executable)) { throw 'Build first: .\scripts\build.ps1' }
& $executable @SimulatorArguments
if ($LASTEXITCODE -ne 0) { throw "Simulator exited with code $LASTEXITCODE" }
