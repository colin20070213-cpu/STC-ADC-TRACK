param([string]$UV4 = 'D:\pie\SOFTWARE\UV4\UV4.exe')
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$project = Join-Path $root 'MDK\Project_Template.uvproj'
$log = Join-Path $root 'MDK\track-build.log'
if (-not (Test-Path -LiteralPath $UV4)) { throw "Keil executable not found: $UV4" }
$process = Start-Process -FilePath $UV4 -ArgumentList @('-r', ('"'+$project+'"'), '-o', ('"'+$log+'"')) -WindowStyle Hidden -PassThru
if (-not $process.WaitForExit(60000)) { throw "Build still running (PID $($process.Id)); inspect $log" }
$output = Get-Content -LiteralPath $log -Raw
Write-Output $output
if ($output -notmatch '0 Error\(s\), 0 Warning\(s\)') { throw 'Firmware build did not finish cleanly' }
& python (Join-Path $PSScriptRoot 'verify_firmware.py')
if ($LASTEXITCODE -ne 0) { throw 'Firmware address verification failed' }
