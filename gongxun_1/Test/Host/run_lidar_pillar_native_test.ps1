param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectDir = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$compilerPath = (Get-Command $Compiler -ErrorAction Stop).Source
$env:PATH = (Split-Path -Parent $compilerPath) + ';' + $env:PATH
$outputDir = Join-Path $projectDir 'build\LidarPillarNativeHostCheck'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$exe = Join-Path $outputDir 'lidar_pillar_native_host_test.exe'
$sources = @(
    (Join-Path $projectDir 'BSP\Src\lidar.c'),
    (Join-Path $projectDir 'APP\Src\map_tf2d.c'),
    (Join-Path $projectDir 'APP\Src\map_obs.c'),
    (Join-Path $PSScriptRoot 'lidar_pillar_native_host_test.c')
)
$nativeSource = Join-Path $projectDir 'Test\Src\lidar_pillar_test.c'
if (Test-Path -LiteralPath $nativeSource) { $sources += $nativeSource }
& $compilerPath -std=c11 -O2 -Wall -Wextra -Werror -DLIDAR_PILLAR_HOST_TEST `
    -I (Join-Path $PSScriptRoot 'pillar_native_include') `
    -I (Join-Path $projectDir 'BSP\Inc') -I (Join-Path $projectDir 'APP\Inc') `
    -I (Join-Path $projectDir 'Test\Inc') @sources -lm -o $exe
if ($LASTEXITCODE -ne 0) { throw "原始点小柱测试编译失败：$LASTEXITCODE" }
& $exe
if ($LASTEXITCODE -ne 0) { throw "原始点小柱测试失败：$LASTEXITCODE" }
