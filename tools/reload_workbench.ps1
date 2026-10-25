# 本机离线工作台重载。默认只显示计划；用户显式-Apply才执行。
# 不连接树莓派/串口/SWD，不下载固件；仅处理本工程已登记的两个离线服务。
param([switch]$Apply)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path $PSScriptRoot -Parent
$taskRuntime = Join-Path $taskRoot 'output/workbench'
$taskPython = Join-Path $taskRoot '.venv/Scripts/python.exe'
$taskLibrary = Join-Path $taskRuntime 'native.dll'
$taskState = Invoke-RestMethod 'http://127.0.0.1:8090/state' -TimeoutSec 3
if (-not $taskState.available -or $taskState.domain -ne 'simulation' -or $taskState.real_connected -or
    $taskState.mechanism.busy -or $taskState.simulation.mode -in @('RUNNING', 'MANUAL') -or
    $taskState.owner.remaining_s -gt 0) {
    throw '请先结束模拟动作并释放工作台控制权；没有结束进程。'
}
$taskSaved = Get-Content (Join-Path $taskRuntime 'processes.json') -Raw | ConvertFrom-Json
$taskWorkers = @()
foreach ($taskPort in @(8090, 8000)) {
    $taskListener = @(Get-NetTCPConnection -LocalPort $taskPort -State Listen)
    if ($taskListener.Count -ne 1) { throw "端口$taskPort的服务不唯一，保持现状" }
    $taskWorker = Get-CimInstance Win32_Process -Filter "ProcessId=$($taskListener[0].OwningProcess)"
    $taskExpected = if ($taskPort -eq 8090) { $taskSaved.core_pid } else { $taskSaved.web_pid }
    $taskModule = if ($taskPort -eq 8090) { 'robot.workbench.core' } else { 'robot.web.app' }
    $taskParent = Get-CimInstance Win32_Process -Filter "ProcessId=$taskExpected"
    if ($taskWorker.ParentProcessId -ne $taskExpected -or
        $taskWorker.CommandLine -notlike "*-m $taskModule *" -or
        $taskParent.CommandLine -notlike "*$taskRoot*" -or
        $taskParent.CommandLine -notlike "*-m $taskModule *") {
        throw "端口$taskPort进程不属于本工程登记的离线服务，保持现状"
    }
    $taskWorkers += $taskWorker
}
if (-not $Apply) {
    [pscustomobject]@{ action='仅预览；没有结束进程'; core=$taskWorkers[0].ProcessId;
        web=$taskWorkers[1].ProcessId; runtime=$taskRuntime; device_access=$false } | ConvertTo-Json
    return
}
$taskBackup = Join-Path $taskRuntime ('reload_' + (Get-Date -Format 'yyyyMMdd_HHmmss'))
New-Item -ItemType Directory -Path $taskBackup -ErrorAction Stop | Out-Null
foreach ($taskName in @('workspace.json', 'mechanism_poses.json', 'processes.json')) {
    $taskFile = Join-Path $taskRuntime $taskName
    if (Test-Path -LiteralPath $taskFile) { Copy-Item -LiteralPath $taskFile -Destination (Join-Path $taskBackup $taskName) }
}
$taskState | ConvertTo-Json -Depth 80 | Set-Content (Join-Path $taskBackup 'state_before.json') -Encoding utf8
# 两个进程都已核对身份且均为空闲模型，才执行重载。
foreach ($taskWorker in $taskWorkers) { Stop-Process -Id $taskWorker.ProcessId }
$taskFree = $false
for ($taskTry = 0; $taskTry -lt 30; $taskTry++) {
    if (-not (Get-NetTCPConnection -LocalPort 8090,8000 -State Listen -ErrorAction SilentlyContinue)) {
        $taskFree = $true
        break
    }
    Start-Sleep -Milliseconds 100
}
if (-not $taskFree) { throw '旧离线服务尚未退出；保留备份，不结束其它进程' }
# 先重建正式C主机库，避免Python配置结构更新后加载旧ABI；不包含硬件驱动。
& $taskPython -B (Join-Path $PSScriptRoot 'build_odom.py') --output $taskLibrary
if ($LASTEXITCODE -ne 0) { throw '新主机模型库构建失败；未启动服务，不连接实车' }
$taskPreviousPythonPath = $env:PYTHONPATH
try {
    $env:PYTHONPATH = Join-Path $taskRoot 'pi'
    $taskCore = Start-Process -FilePath $taskPython -ArgumentList @('-B','-m','robot.workbench.core',
        '--library',"`"$taskLibrary`"",'--runtime',"`"$taskRuntime`"") -WorkingDirectory $taskRoot -WindowStyle Hidden -PassThru `
        -RedirectStandardOutput (Join-Path $taskBackup 'core.log') -RedirectStandardError (Join-Path $taskBackup 'core.err')
    $taskReady = $false
    for ($taskTry = 0; $taskTry -lt 60; $taskTry++) {
        try {
            $taskNew = Invoke-RestMethod 'http://127.0.0.1:8090/state' -TimeoutSec 1
            if ($taskNew.available -and $taskNew.mechanism.execution_schema -eq 2) { $taskReady=$true; break }
        } catch { Start-Sleep -Milliseconds 200 }
    }
    if (-not $taskReady) { throw "新离线核心未就绪，检查$taskBackup/core.err；不会连接实车" }
    $taskWeb = Start-Process -FilePath $taskPython -ArgumentList @('-B','-m','robot.web.app','--port','8000') `
        -WorkingDirectory $taskRoot -WindowStyle Hidden -PassThru `
        -RedirectStandardOutput (Join-Path $taskBackup 'web.log') -RedirectStandardError (Join-Path $taskBackup 'web.err')
    @{ core_pid=$taskCore.Id; web_pid=$taskWeb.Id; url='http://127.0.0.1:8000/' } |
        ConvertTo-Json | Set-Content (Join-Path $taskRuntime 'processes.json') -Encoding utf8
    [pscustomobject]@{ reloaded=$true; backup=$taskBackup; device_access=$false;
        zero_valid=$taskNew.mechanism.zero_valid; busy=$taskNew.mechanism.busy;
        draft_preserved=(($taskState.draft | ConvertTo-Json -Depth 30 -Compress) -ceq
                         ($taskNew.draft | ConvertTo-Json -Depth 30 -Compress));
        core=$taskCore.Id; web=$taskWeb.Id } | ConvertTo-Json
} finally {
    $env:PYTHONPATH = $taskPreviousPythonPath
}
