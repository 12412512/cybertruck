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
foreach ($taskId in @($taskProcesses.core, $taskProcesses.web)) {
    $taskProcess = Get-CimInstance Win32_Process -Filter "ProcessId = $taskId"
    if ($null -eq $taskProcess) { continue }
    # PID可能被系统复用，必须同时核对本项目运行目录和模块名。
    if (-not $taskProcess.CommandLine.Contains($taskRuntime) -or
        $taskProcess.CommandLine -notmatch 'robot\.(main|web\.app)') {
        throw "进程身份不符，保留进程：$taskId"
    }
    Stop-Process -Id $taskId -ErrorAction Stop
}
Write-Output '本机预览进程已停止；未修改树莓派服务'
