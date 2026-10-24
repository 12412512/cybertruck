# 仅停止由本工程启动脚本记录的离线核心和网页，不访问真实设备。
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path $PSScriptRoot -Parent
$taskRecord = Join-Path $taskRoot 'output/workbench/processes.json'
$taskProcesses = Get-Content -LiteralPath $taskRecord -Raw | ConvertFrom-Json
foreach ($taskEntry in @(@{pid=$taskProcesses.web_pid; module='robot.web.app'},
                         @{pid=$taskProcesses.core_pid; module='robot.workbench.core'})) {
    $taskProcess = Get-CimInstance Win32_Process -Filter "ProcessId=$($taskEntry.pid)"
    if (-not $taskProcess) { continue }
    if ($taskProcess.CommandLine -notlike "*$taskRoot*" -or
        $taskProcess.CommandLine -notlike "* $($taskEntry.module) *") {
        throw "PID=$($taskEntry.pid)身份不符，拒绝停止。"
    }
    $taskChildren = Get-CimInstance Win32_Process -Filter "ParentProcessId=$($taskProcess.ProcessId)"
    foreach ($taskChild in $taskChildren) {
        if ($taskChild.CommandLine -like "* $($taskEntry.module) *") {
            Stop-Process -Id $taskChild.ProcessId -ErrorAction SilentlyContinue
        }
    }
    Stop-Process -Id $taskProcess.ProcessId -ErrorAction SilentlyContinue
}
Write-Output '本工程离线工作台已停止。'
