param(
    [ValidateSet('preview', 'observer')]
    [string]$Name = 'preview'
)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskRuntime = Join-Path $taskRoot "output\$Name"
$taskRecord = Join-Path $taskRuntime 'processes.json'
if (-not (Test-Path -LiteralPath $taskRecord)) { return }
$taskProcesses = Get-Content -LiteralPath $taskRecord -Raw | ConvertFrom-Json
$taskInventory = @(Get-CimInstance Win32_Process)
$taskPending = @($taskProcesses.core, $taskProcesses.web)
$taskVerified = @()
while ($taskPending.Count -gt 0) {
    $taskId = $taskPending[0]
    $taskPending = @($taskPending | Select-Object -Skip 1)
    $taskProcess = $taskInventory | Where-Object { $_.ProcessId -eq $taskId }
    if ($null -eq $taskProcess -or $taskId -in $taskVerified) { continue }
    # PID可能被系统复用，必须同时核对本项目运行目录和模块名。
    if (-not $taskProcess.CommandLine.Contains($taskRuntime) -or
        $taskProcess.CommandLine -notmatch 'robot\.(main|web\.app)') {
        throw "进程身份不符，保留进程：$taskId"
    }
    $taskVerified += $taskId
    # Windows虚拟环境启动器有独立Python子进程，不能只停止外层留下旧服务。
    $taskPending += @($taskInventory | Where-Object {
        $_.ParentProcessId -eq $taskId -and $_.Name -in @('python.exe', 'pythonw.exe') } |
        ForEach-Object { $_.ProcessId })
}
for ($taskIndex = $taskVerified.Count - 1; $taskIndex -ge 0; $taskIndex--) {
    $taskId = $taskVerified[$taskIndex]
    $taskCurrent = Get-CimInstance Win32_Process -Filter "ProcessId = $taskId"
    if ($null -eq $taskCurrent) { continue }
    if (-not $taskCurrent.CommandLine.Contains($taskRuntime) -or
        $taskCurrent.CommandLine -notmatch 'robot\.(main|web\.app)') {
        throw "停止前进程身份已变化，保留进程：$taskId"
    }
    Stop-Process -Id $taskId -ErrorAction Stop
}
Write-Output '本机预览进程已停止；未修改树莓派服务'
