param([string]$Compiler = '')

$ErrorActionPreference = 'Stop'
$projectDir = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$buildDir = Join-Path $projectDir 'build\host_checks'
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

# 使用电脑上的原生C编译器，不使用ARM编译器运行主机测试。
if (!$Compiler) {
    $gccCommand = Get-Command gcc -ErrorAction SilentlyContinue
    if ($gccCommand) { $Compiler = $gccCommand.Source }
}
if (!$Compiler) {
    $clionGcc = Get-ChildItem 'D:\CLion\*\bin\mingw\bin\gcc.exe' -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if ($clionGcc) { $Compiler = $clionGcc.FullName }
}
if (!$Compiler -or !(Test-Path -LiteralPath $Compiler)) {
    throw '请通过 -Compiler 指定电脑上用于Windows的gcc.exe。'
}
$Compiler = (Resolve-Path -LiteralPath $Compiler).Path
$savedPath = $env:Path
$env:Path = (Split-Path -Parent $Compiler) + ';' + $env:Path

function Run-Check {
    param([string]$Name, [string[]]$Sources, [string]$HeadDir, [string[]]$ExtraFlags = @())
    $exePath = Join-Path $buildDir ($Name + '.exe')
    $compileArgs = @('-std=c11', '-Wall', '-Wextra', '-Werror') + $ExtraFlags
    foreach ($include in @($HeadDir, 'APP/Inc', 'BSP/Inc', 'Test/Inc', 'Control/Inc')) {
        $compileArgs += '-I' + (Join-Path $projectDir $include)
    }
    foreach ($source in $Sources) {
        if ([IO.Path]::IsPathRooted($source)) { $compileArgs += $source }
        else { $compileArgs += Join-Path $projectDir $source }
    }
    $compileArgs += @('-lm', '-o', $exePath)
    & $Compiler @compileArgs
    if ($LASTEXITCODE -ne 0) { throw "$Name 编译失败。" }
    & $exePath | Tee-Object -FilePath (Join-Path $buildDir ($Name + '.txt'))
    if ($LASTEXITCODE -ne 0) { throw "$Name 验证失败，请查看上面的FAIL项。" }
}

try {
    Run-Check 'lidar' @('Test/Host/test_lidar.c') 'Test/Host/lidar_headfile'
    Run-Check 'cluster' @('Test/Host/test_cluster.c', 'APP/Src/obstacle_find.c', 'APP/Src/tf2d.c') 'Test/Host/cluster_headfile'
    Run-Check 'cluster_mask' @('Test/Host/test_cluster.c', 'APP/Src/obstacle_find.c', 'APP/Src/tf2d.c') 'Test/Host/cluster_headfile' @('-DLIDAR_WHEEL_MASK_ENABLE=1', '-O2')
    $mapSources = @('APP/Src/map_data.c', 'APP/Src/map_update.c', 'APP/Src/map_plan.c', 'APP/Src/track_map.c', 'APP/Src/map_lidar.c')
    Run-Check 'map' (@('Test/Host/test_map.c', 'Test/Src/map_stage_test.c') + $mapSources) 'Test/Host/map_headfile'
    Run-Check 'pipeline' (@('Test/Host/test_pipeline.c', 'APP/Src/obstacle_find.c', 'APP/Src/tf2d.c') + $mapSources) 'Test/Host/map_headfile'

    # 编译完整的任务源码，只由test_task提供外设替身和可控扫描时序。
    Run-Check 'task' @('Test/Host/test_task.c', 'Control/Src/task.c', 'APP/Src/map_data.c', 'APP/Src/map_update.c', 'APP/Src/map_lidar.c', 'APP/Src/track_map.c') 'Test/Host/task_headfile'
    Write-Output 'ALL HOST CHECKS PASS'
    Write-Output "验证日志：$buildDir"
} finally {
    $env:Path = $savedPath
}
