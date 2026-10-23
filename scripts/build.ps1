param(
    [ValidateSet('Release', 'Debug')][string]$Configuration = 'Release',
    [switch]$Headless
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$vswherePath = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
if (-not (Test-Path -LiteralPath $vswherePath)) { throw 'Visual Studio C++ Build Tools are required.' }
$vsPath = & $vswherePath -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsPath) { throw 'No Visual Studio installation with C++ tools found.' }
$devShellPath = Join-Path $vsPath 'Common7\Tools\Microsoft.VisualStudio.DevShell.dll'
Import-Module $devShellPath
Enter-VsDevShell -VsInstallPath $vsPath -SkipAutomaticLocation -DevCmdArguments '-arch=x64 -host_arch=x64'
$cmakeDir = Join-Path $vsPath 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin'
$ninjaDir = Join-Path $vsPath 'Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja'
if (Test-Path -LiteralPath $cmakeDir) { $env:PATH = "$cmakeDir;$env:PATH" }
if (Test-Path -LiteralPath $ninjaDir) { $env:PATH = "$ninjaDir;$env:PATH" }
$preset = $Configuration.ToLowerInvariant()
$viewer = if ($Headless) { 'OFF' } else { 'ON' }
Push-Location $projectRoot
try {
    & cmake --preset $preset "-DVSIM_BUILD_VIEWER=$viewer"
    if ($LASTEXITCODE -ne 0) { throw 'CMake configuration failed.' }
    & cmake --build --preset $preset --parallel
    if ($LASTEXITCODE -ne 0) { throw 'Build failed.' }
    Write-Host "Ready: $projectRoot\build\$preset\bin"
} finally { Pop-Location }
