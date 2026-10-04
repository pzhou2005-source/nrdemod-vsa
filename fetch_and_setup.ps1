<#
.SYNOPSIS
  One shot setup of the nrdemod / 89600 correlation bench on this Windows PC.

.DESCRIPTION
  Copies the bundle from the Linux build host, unpacks it, builds nrdemod.dll,
  creates the Python virtual environment and runs the self test.

  Run it from a normal PowerShell prompt:

      powershell -ExecutionPolicy Bypass -File .\fetch_and_setup.ps1

.PARAMETER Source
  Where to get the bundle from. Either an SSH location handled by scp
  (default) or a local/UNC path to nrdemod-vsa.zip.

.PARAMETER Destination
  Where to install. Defaults to C:\nrdemod-vsa.
#>
[CmdletBinding()]
param(
  [string]$Source = "socbn505:/home/pengzhou/nrdemod-vsa.zip",
  [string]$Destination = "C:\nrdemod-vsa",
  [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"

function Require-Command($name, $hint) {
  if (-not (Get-Command $name -ErrorAction SilentlyContinue)) {
    throw "'$name' is not on PATH. $hint"
  }
}

$zip = Join-Path $env:TEMP "nrdemod-vsa.zip"

if ($Source -match '^[^\\/:]+:[^\\]' -and $Source -notmatch '^[A-Za-z]:\\') {
  Require-Command scp "Install OpenSSH client (Settings > Apps > Optional Features)."
  Write-Host "fetching $Source ..." -ForegroundColor Cyan
  & scp $Source $zip
  if ($LASTEXITCODE -ne 0) { throw "scp failed - check that you can 'ssh socbn505'." }
} else {
  Write-Host "copying $Source ..." -ForegroundColor Cyan
  Copy-Item -LiteralPath $Source -Destination $zip -Force
}

if (Test-Path $Destination) {
  Write-Host "removing the previous install at $Destination" -ForegroundColor Yellow
  Remove-Item -Recurse -Force $Destination
}

$parent = Split-Path -Parent $Destination
Write-Host "unpacking to $Destination ..." -ForegroundColor Cyan
Expand-Archive -LiteralPath $zip -DestinationPath $parent -Force
$extracted = Join-Path $parent "nrdemod-vsa"
if ($extracted -ne $Destination) { Rename-Item -LiteralPath $extracted -NewName (Split-Path -Leaf $Destination) }

if ($SkipBuild) {
  Write-Host "`nUnpacked. Run build.bat when you are ready." -ForegroundColor Green
  exit 0
}

Require-Command cmake "Install CMake and tick 'Add to PATH'."
if (-not (Get-Command py -ErrorAction SilentlyContinue) -and
    -not (Get-Command python -ErrorAction SilentlyContinue)) {
  throw "No Python found. Install Python 3.8+ and tick 'Add to PATH'."
}

Write-Host "`nbuilding ..." -ForegroundColor Cyan
Push-Location $Destination
try {
  & cmd /c "build.bat"
  if ($LASTEXITCODE -ne 0) { throw "build.bat failed with exit code $LASTEXITCODE" }
} finally {
  Pop-Location
}

Write-Host @"

Done.

  Workspace : $Destination\nrdemod-vsa.code-workspace
  Probe VSA : $Destination\.venv\Scripts\python.exe $Destination\tools\vsa_link.py --probe --host 127.0.0.1
  Self test : $Destination\.venv\Scripts\python.exe $Destination\tools\correlate.py selftest

Open the workspace in a LOCAL VS Code window (File > New Window, then
File > Open Workspace from File).
"@ -ForegroundColor Green
