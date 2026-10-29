# 启动本机离线工作台；不连接树莓派、串口或烧录器。
param([int]$Port = 8000)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path $PSScriptRoot -Parent
$taskPython = Join-Path $taskRoot '.venv/Scripts/python.exe'
$taskOutput = Join-Path $taskRoot 'output/workbench'
$taskLibrary = Join-Path $taskOutput 'native.dll'
$taskWebPort = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue
$taskCorePort = Get-NetTCPConnection -LocalPort 8090 -State Listen -ErrorAction SilentlyContinue
if ($taskWebPort -or $taskCorePort) {
    throw "端口$Port 或8090已被占用；请先检查已有服务，本脚本不自动结束其他进程。"
}
New-Item -ItemType Directory -Path $taskOutput -Force | Out-Null
& $taskPython -B (Join-Path $PSScriptRoot 'build_odom.py') --output $taskLibrary
if ($LASTEXITCODE -ne 0) { throw '离线C模型库构建失败' }
$env:PYTHONPATH = Join-Path $taskRoot 'pi'
$taskCore = Start-Process -FilePath $taskPython -ArgumentList @('-B', '-m', 'robot.workbench.core',
    '--library', "`"$taskLibrary`"", '--runtime', "`"$taskOutput`"") -WorkingDirectory $taskRoot -WindowStyle Hidden -PassThru `
    -RedirectStandardOutput (Join-Path $taskOutput 'core.log') -RedirectStandardError (Join-Path $taskOutput 'core.err')
try {
    $taskReady = $false
    for ($taskTry = 0; $taskTry -lt 30; $taskTry++) {
        try {
            $taskState = Invoke-RestMethod 'http://127.0.0.1:8090/state' -TimeoutSec 1
            if ($taskState.available -and $taskState.domain -eq 'simulation') { $taskReady = $true; break }
        } catch { Start-Sleep -Milliseconds 100 }
    }
    if (-not $taskReady) { throw '离线核心未就绪，请检查output/workbench/core.err' }
    $taskWeb = Start-Process -FilePath $taskPython -ArgumentList @('-B', '-m', 'robot.web.app',
        '--port', "$Port") -WorkingDirectory $taskRoot -WindowStyle Hidden -PassThru `
        -RedirectStandardOutput (Join-Path $taskOutput 'web.log') -RedirectStandardError (Join-Path $taskOutput 'web.err')
    @{core_pid=$taskCore.Id; web_pid=$taskWeb.Id; url="http://127.0.0.1:$Port/"} |
        ConvertTo-Json | Set-Content (Join-Path $taskOutput 'processes.json') -Encoding utf8
    Write-Output "离线工作台：http://127.0.0.1:$Port/；核心PID=$($taskCore.Id)，网页PID=$($taskWeb.Id)"
} catch {
    Stop-Process -Id $taskCore.Id -ErrorAction SilentlyContinue
    throw
}
