param(
    [string]$Keil = 'D:\Kiles\UV4\UV4.exe',
    [string]$Python = (Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe')
)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
if (-not (Test-Path -LiteralPath $Keil -PathType Leaf)) { throw "Keil 不存在：$Keil" }
if (-not (Test-Path -LiteralPath $Python -PathType Leaf)) { throw "Python 不存在：$Python" }
& $Python (Join-Path $PSScriptRoot 'check_project.py')
if ($LASTEXITCODE -ne 0) { throw '工程检查失败，未执行编译' }
$taskOutput = Join-Path $taskRoot 'output\keil'
New-Item -ItemType Directory -Path $taskOutput -Force | Out-Null
$taskProject = Join-Path $taskRoot 'firmware\MDK-ARM\Car.uvprojx'
$taskLog = Join-Path $taskOutput 'build.log'
# 先清空上次日志，避免将旧的零错误结果当作本次成功。
[System.IO.File]::WriteAllText($taskLog, '')
$taskArgs = '-r "{0}" -t Car -o "{1}"' -f $taskProject, $taskLog
$taskStarted = Get-Date
$taskProcess = Start-Process -FilePath $Keil -ArgumentList $taskArgs -WorkingDirectory (Split-Path -Parent $taskProject) -WindowStyle Hidden -Wait -PassThru
$taskText = Get-Content -LiteralPath $taskLog -Raw -Encoding UTF8
Write-Output $taskText
if ($taskProcess.ExitCode -ne 0 -or $taskText -notmatch '0 Error\(s\), 0 Warning\(s\)') {
    throw "Keil 编译未通过，退出码：$($taskProcess.ExitCode)"
}
# 编译后再次核对，防止IDE保存把保护项改回默认值。
& $Python (Join-Path $PSScriptRoot 'check_project.py')
if ($LASTEXITCODE -ne 0) { throw '编译后工程保护项发生变化，结果不作为有效基线' }
$taskHex = Join-Path $taskRoot 'firmware\MDK-ARM\Objects\Car.hex'
foreach ($taskName in @('Car.axf', 'Car.hex')) {
    $taskArtifact = Get-Item -LiteralPath (Join-Path $taskRoot "firmware\MDK-ARM\Objects\$taskName")
    if ($taskArtifact.LastWriteTime -lt $taskStarted.AddSeconds(-2)) { throw "未生成本次编译产物：$taskName" }
}
Get-FileHash -Algorithm SHA256 -LiteralPath $taskHex
