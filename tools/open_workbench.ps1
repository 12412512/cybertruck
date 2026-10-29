# 用户双击入口：恢复PC服务，设备连接由网页明确触发。
param([switch]$NoOpen)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path $PSScriptRoot -Parent
$taskWeb = Get-NetTCPConnection -LocalPort 8000 -State Listen -ErrorAction SilentlyContinue
$taskCore = Get-NetTCPConnection -LocalPort 8090 -State Listen -ErrorAction SilentlyContinue
if (-not $taskWeb -and -not $taskCore) {
    & (Join-Path $PSScriptRoot 'start_workbench.ps1')
} else {
    foreach ($taskEntry in @(@{Listener=$taskWeb; Module='robot.web.app'},@{Listener=$taskCore; Module='robot.workbench.core'})) {
        if ($taskEntry.Listener) {
            $taskWorker = Get-CimInstance Win32_Process -Filter "ProcessId=$($taskEntry.Listener.OwningProcess)"
            if ($taskWorker.CommandLine -notlike "*-m $($taskEntry.Module)*") {
                throw 'A workbench port belongs to another application; unchanged.'
            }
        }
    }
    if (-not $taskWeb -or -not $taskCore) {
        $taskOutput = Join-Path $taskRoot 'output/workbench'
        $taskPython = Join-Path $taskRoot '.venv/Scripts/python.exe'
        $taskProcessFile = Join-Path $taskOutput 'processes.json'
        $taskSaved = Get-Content -LiteralPath $taskProcessFile -Raw | ConvertFrom-Json
        $taskOldPath = $env:PYTHONPATH
        try {
            $env:PYTHONPATH = Join-Path $taskRoot 'pi'
            if (-not $taskCore) {
                $taskLibrary = Join-Path $taskOutput 'native.dll'
                & $taskPython -B (Join-Path $PSScriptRoot 'build_odom.py') --output $taskLibrary
                if ($LASTEXITCODE -ne 0) { throw 'Offline model build failed.' }
                $taskNew = Start-Process -FilePath $taskPython -ArgumentList @('-B','-m','robot.workbench.core',
                    '--library',"`"$taskLibrary`"",'--runtime',"`"$taskOutput`"") -WorkingDirectory $taskRoot -WindowStyle Hidden -PassThru `
                    -RedirectStandardOutput (Join-Path $taskOutput 'core_restore.log') -RedirectStandardError (Join-Path $taskOutput 'core_restore.err')
                $taskSaved.core_pid=$taskNew.Id
            }
            if (-not $taskWeb) {
                $taskNew = Start-Process -FilePath $taskPython -ArgumentList @('-B','-m','robot.web.app','--port','8000') `
                    -WorkingDirectory $taskRoot -WindowStyle Hidden -PassThru `
                    -RedirectStandardOutput (Join-Path $taskOutput 'web_restore.log') -RedirectStandardError (Join-Path $taskOutput 'web_restore.err')
                $taskSaved.web_pid=$taskNew.Id
            }
            $taskSaved | ConvertTo-Json | Set-Content -LiteralPath $taskProcessFile -Encoding utf8
        } finally { $env:PYTHONPATH=$taskOldPath }
    }
}
& (Join-Path $PSScriptRoot 'start_connections.ps1')
if (-not $NoOpen) { Start-Process 'http://127.0.0.1:8000/' }
