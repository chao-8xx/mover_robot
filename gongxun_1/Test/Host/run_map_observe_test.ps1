param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectDir = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$compilerPath = (Get-Command $Compiler -ErrorAction Stop).Source
$env:PATH = (Split-Path -Parent $compilerPath) + ';' + $env:PATH
$outputDir = Join-Path $projectDir 'build\MapObserveHostCheck'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$sources = @('map_obs.c','map_tf2d.c','map_track.c','map_data.c','map_update.c','map_lidar.c','map_plan.c','map_pose.c','map_nav.c') |
    ForEach-Object { Join-Path $projectDir ('APP\Src\' + $_) }
$exe = Join-Path $outputDir 'map_observe_host_test.exe'
& $compilerPath -std=c11 -O2 -Wall -Wextra -Werror `
    -I (Join-Path $PSScriptRoot 'perception_include') `
    -I (Join-Path $projectDir 'APP\Inc') -I (Join-Path $projectDir 'BSP\Inc') `
    @sources (Join-Path $PSScriptRoot 'map_observe_host_test.c') -lm -o $exe
if ($LASTEXITCODE -ne 0) { throw "方向观察测试编译失败：$LASTEXITCODE" }
& $exe
if ($LASTEXITCODE -ne 0) { throw "方向观察测试失败：$LASTEXITCODE" }
