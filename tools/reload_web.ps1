# 仅重载本工程8000网页网关，不重启8090模型、树莓派、STM32或雷达核心。
# 默认只核对计划，显式-Apply后才结束已核对的网页进程。
param([switch]$Apply)
# 在执行外部查询前锁存明确的绑定结果，预览输出也显示是否收到参数。
$taskApplyRequested = $Apply.IsPresent
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskRuntime = Join-Path $taskRoot 'output/workbench'
$taskProcessFile = Join-Path $taskRuntime 'processes.json'
$taskSaved = Get-Content -LiteralPath $taskProcessFile -Raw | ConvertFrom-Json
$taskState = Invoke-RestMethod 'http://127.0.0.1:8090/state' -TimeoutSec 3
if (-not $taskState.available -or $taskState.domain -ne 'simulation' -or
    $taskState.mechanism.busy -or $taskState.program.busy -or $taskState.transfer.busy -or
    $taskState.execution_check.status -eq 'RUNNING' -or
    $taskState.simulation.mode -in @('RUNNING', 'MANUAL') -or $taskState.owner.remaining_s -gt 0) {
    throw 'Please stop the model and release its control lease first; no process was stopped.'
}
$taskListeners = @(Get-NetTCPConnection -LocalPort 8000 -State Listen)
if ($taskListeners.Count -ne 1) { throw 'Web listener is not unique; unchanged.' }
$taskWorker = Get-CimInstance Win32_Process -Filter "ProcessId=$($taskListeners[0].OwningProcess)"
$taskParent = Get-CimInstance Win32_Process -Filter "ProcessId=$($taskSaved.web_pid)"
if ($taskWorker.ParentProcessId -ne $taskSaved.web_pid -or
    $taskWorker.CommandLine -notlike '*-m robot.web.app --port 8000*' -or
    $taskParent.CommandLine -notlike "*$taskRoot*" -or
    $taskParent.CommandLine -notlike '*-m robot.web.app --port 8000*') {
    throw 'Web process identity mismatch; unchanged.'
}
if (-not $taskApplyRequested) {
    [pscustomobject]@{preview=$true; apply_received=$taskApplyRequested; web_worker=$taskWorker.ProcessId;
        core_unchanged=$taskSaved.core_pid; device_commands=0} | ConvertTo-Json
    return
}
$taskBackup = Join-Path $taskRuntime ('web_reload_' + (Get-Date -Format 'yyyyMMdd_HHmmss'))
New-Item -ItemType Directory -Path $taskBackup | Out-Null
Copy-Item -LiteralPath $taskProcessFile -Destination (Join-Path $taskBackup 'processes_before.json')
$taskState | ConvertTo-Json -Depth 80 | Set-Content (Join-Path $taskBackup 'core_before.json') -Encoding utf8
$taskHashes = @{}
foreach ($taskName in @('workspace.json','mechanism_poses.json')) {
    $taskFile = Join-Path $taskRuntime $taskName
    if (Test-Path -LiteralPath $taskFile) {
        Copy-Item -LiteralPath $taskFile -Destination (Join-Path $taskBackup $taskName)
        $taskHashes[$taskName] = (Get-FileHash -LiteralPath $taskFile -Algorithm SHA256).Hash
    }
}
Stop-Process -Id $taskWorker.ProcessId
for ($taskTry = 0; $taskTry -lt 30; $taskTry++) {
    if (-not (Get-NetTCPConnection -LocalPort 8000 -State Listen -ErrorAction SilentlyContinue)) { break }
    Start-Sleep -Milliseconds 100
}
if (Get-NetTCPConnection -LocalPort 8000 -State Listen -ErrorAction SilentlyContinue) {
    throw 'Previous Web process has not released the port; no other process will be stopped.'
}
$taskOldPath = $env:PYTHONPATH
try {
    $env:PYTHONPATH = Join-Path $taskRoot 'pi'
    $taskWeb = Start-Process -FilePath (Join-Path $taskRoot '.venv/Scripts/python.exe') `
        -ArgumentList @('-B','-m','robot.web.app','--port','8000') -WorkingDirectory $taskRoot `
        -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $taskBackup 'web.log') `
        -RedirectStandardError (Join-Path $taskBackup 'web.err')
    $taskSaved.web_pid = $taskWeb.Id
    $taskSaved | ConvertTo-Json | Set-Content -LiteralPath $taskProcessFile -Encoding utf8
} finally { $env:PYTHONPATH = $taskOldPath }
$taskReady = $false
for ($taskTry = 0; $taskTry -lt 30; $taskTry++) {
    try {
        $taskLidar = Invoke-RestMethod 'http://127.0.0.1:8000/api/lidar/state' -TimeoutSec 2
        if ($taskLidar.domain -eq 'physical_lidar') { $taskReady=$true; break }
    } catch { Start-Sleep -Milliseconds 200 }
}
if (-not $taskReady) { throw "New Web gateway not ready; inspect $taskBackup/web.err" }
$taskPreserved = $true
foreach ($taskName in $taskHashes.Keys) {
    if ((Get-FileHash -LiteralPath (Join-Path $taskRuntime $taskName) -Algorithm SHA256).Hash -ne $taskHashes[$taskName]) {
        $taskPreserved = $false
    }
}
[pscustomobject]@{web_reloaded=$true; backup=$taskBackup; core_unchanged=$taskSaved.core_pid;
    drafts_preserved=$taskPreserved; device_commands=0; lidar_connected=$taskLidar.connected} | ConvertTo-Json
