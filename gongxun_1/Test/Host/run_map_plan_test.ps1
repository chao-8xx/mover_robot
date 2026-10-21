param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectDir = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$compilerPath = (Get-Command $Compiler -ErrorAction Stop).Source
$env:PATH = (Split-Path -Parent $compilerPath) + ';' + $env:PATH
$outputDir = Join-Path $projectDir 'build\MapPlanHostCheck'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$exe = Join-Path $outputDir 'map_plan_host_test.exe'
& $compilerPath -std=c11 -O2 -Wall -Wextra -Werror `
    -I (Join-Path $PSScriptRoot 'map_include') -I (Join-Path $projectDir 'APP\Inc') `
    (Join-Path $projectDir 'APP\Src\map_track.c') `
    (Join-Path $projectDir 'APP\Src\map_data.c') `
    (Join-Path $projectDir 'APP\Src\map_update.c') `
    (Join-Path $projectDir 'APP\Src\map_plan.c') `
    (Join-Path $PSScriptRoot 'map_plan_host_test.c') -lm -o $exe
if ($LASTEXITCODE -ne 0) { throw "路径规划测试编译失败：$LASTEXITCODE" }
& $exe
if ($LASTEXITCODE -ne 0) { throw "路径规划测试失败：$LASTEXITCODE" }
