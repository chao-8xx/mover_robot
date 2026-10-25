param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectDir = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$compilerPath = (Get-Command $Compiler -ErrorAction Stop).Source
$env:PATH = (Split-Path -Parent $compilerPath) + ';' + $env:PATH
$outputDir = Join-Path $projectDir 'build\OdomHostCheck'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$exe = Join-Path $outputDir 'odom_host_test.exe'
& $compilerPath -std=c11 -O2 -Wall -Wextra -Werror `
    -I (Join-Path $PSScriptRoot 'odom_include') `
    -I (Join-Path $projectDir 'BSP\Inc') -I (Join-Path $projectDir 'APP\Inc') `
    -I (Join-Path $projectDir 'Test\Inc') `
    (Join-Path $projectDir 'BSP\Src\Emm_V5.c') `
    (Join-Path $projectDir 'BSP\Src\step_motor.c') `
    (Join-Path $projectDir 'BSP\Src\hwt101.c') `
    (Join-Path $projectDir 'BSP\Src\key_led.c') `
    (Join-Path $projectDir 'APP\Src\pid.c') `
    (Join-Path $projectDir 'APP\Src\chassis.c') `
    (Join-Path $projectDir 'APP\Src\map_pose.c') `
    (Join-Path $projectDir 'APP\Src\map_track.c') `
    (Join-Path $projectDir 'APP\Src\map_data.c') `
    (Join-Path $projectDir 'APP\Src\map_update.c') `
    (Join-Path $projectDir 'APP\Src\map_plan.c') `
    (Join-Path $projectDir 'APP\Src\map_tf2d.c') `
    (Join-Path $projectDir 'APP\Src\map_obs.c') `
    (Join-Path $projectDir 'APP\Src\map_lidar.c') `
    (Join-Path $projectDir 'APP\Src\map_nav.c') `
    (Join-Path $projectDir 'Test\Src\map_stage_test.c') `
    (Join-Path $PSScriptRoot 'odom_host_test.c') -lm -o $exe
if ($LASTEXITCODE -ne 0) { throw "里程计测试编译失败：$LASTEXITCODE" }
& $exe
if ($LASTEXITCODE -ne 0) { throw "里程计测试失败：$LASTEXITCODE" }
