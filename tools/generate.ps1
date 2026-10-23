param(
    [string]$CubeMX = 'D:\STM\STM32CubeMX.exe',
    [string]$Python = (Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe')
)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskInstall = Split-Path -Parent $CubeMX
$taskJava = Join-Path $taskInstall 'jre\bin\java.exe'
$taskOutput = Join-Path $taskRoot 'output\cubemx'
foreach ($taskTool in @($CubeMX, $taskJava, $Python)) {
    if (-not (Test-Path -LiteralPath $taskTool -PathType Leaf)) { throw "工具不存在：$taskTool" }
}
New-Item -ItemType Directory -Path $taskOutput -Force | Out-Null
$taskScript = Join-Path $taskOutput 'generate.script'
$taskIoc = Join-Path $taskRoot 'firmware\Car.ioc'
$taskText = "config load `"$taskIoc`"`nproject generate`nexit`n"
[System.IO.File]::WriteAllText($taskScript, $taskText, [System.Text.UTF8Encoding]::new($false))
$taskArgs = '-Dfile.encoding=UTF-8 -jar "{0}" -q "{1}"' -f $CubeMX, $taskScript
$taskLog = Join-Path $taskOutput 'generate.log'
$taskErrorLog = Join-Path $taskOutput 'generate-errors.log'
$taskProcess = Start-Process -FilePath $taskJava -ArgumentList $taskArgs -WorkingDirectory $taskInstall -WindowStyle Hidden -Wait -PassThru -RedirectStandardOutput $taskLog -RedirectStandardError $taskErrorLog
$taskText = Get-Content -LiteralPath $taskLog -Raw -Encoding UTF8
# config load 也会输出 OK，必须只判断 project generate 的结果。
$taskGenerate = [regex]::Match($taskText, '(?ms)^project generate\r?\n(?<result>.*?)^exit\s*$')
$taskResult = $taskGenerate.Groups['result'].Value
if ($taskProcess.ExitCode -ne 0 -or -not $taskGenerate.Success -or
    $taskResult -notmatch '(?m)^OK\s*\z' -or $taskResult -match '(?m)^KO\b|\[ERROR\]') {
    throw "CubeMX 生成失败，检查 $taskLog 和 $taskErrorLog"
}
# 内容相同的源文件可能保留修改时间，改以本次生成回执确认 main.c。
$taskMain = Join-Path $taskRoot 'firmware\Core\Src\main.c'
if (-not $taskResult.Contains("Generated code: $taskMain")) { throw '本次日志缺少 main.c 生成回执' }
& $Python (Join-Path $PSScriptRoot 'sync_project.py')
if ($LASTEXITCODE -ne 0) { throw 'Keil 配置同步失败' }
& $Python (Join-Path $PSScriptRoot 'check_project.py')
if ($LASTEXITCODE -ne 0) { throw '生成结果检查失败' }
Write-Output "CubeMX 生成完成，日志：$taskLog"
