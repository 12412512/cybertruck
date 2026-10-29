# 仅启动PC回环连接管理器；启动本身不访问SSH或设备。
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path $PSScriptRoot -Parent
$taskExisting = Get-NetTCPConnection -LocalPort 8094 -State Listen -ErrorAction SilentlyContinue
if ($taskExisting) {
    $taskState = Invoke-RestMethod 'http://127.0.0.1:8094/state' -TimeoutSec 3
    if ($taskState.service -ne 'gc_connections') { throw '8094 belongs to another service.' }
    Write-Output 'Connection manager already running; no device command sent.'
    return
}
$taskOutput = Join-Path $taskRoot 'output/connections'
New-Item -ItemType Directory -Path $taskOutput -Force | Out-Null
$taskOldPath = $env:PYTHONPATH
try {
    $env:PYTHONPATH = Join-Path $taskRoot 'pi'
    $taskProcess = Start-Process -FilePath (Join-Path $taskRoot '.venv/Scripts/python.exe') `
        -ArgumentList @('-B','-m','robot.connection_core') -WorkingDirectory $taskRoot -WindowStyle Hidden -PassThru `
        -RedirectStandardOutput (Join-Path $taskOutput 'core.log') -RedirectStandardError (Join-Path $taskOutput 'core.err')
    @{pid=$taskProcess.Id; port=8094; hardware_access_on_start=$false} | ConvertTo-Json |
        Set-Content (Join-Path $taskOutput 'process.json') -Encoding utf8
} finally { $env:PYTHONPATH = $taskOldPath }
for ($taskTry=0; $taskTry -lt 30; $taskTry++) {
    try {
        $taskState = Invoke-RestMethod 'http://127.0.0.1:8094/state' -TimeoutSec 2
        if ($taskState.service -eq 'gc_connections') { Write-Output 'Connection manager ready; click Connect in Web.'; return }
    } catch { Start-Sleep -Milliseconds 100 }
}
throw 'Connection manager not ready; check output/connections/core.err.'
