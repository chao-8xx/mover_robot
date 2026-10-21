param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectDir = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$compilerPath = (Get-Command $Compiler -ErrorAction Stop).Source
$env:PATH = (Split-Path -Parent $compilerPath) + ';' + $env:PATH
$outputDir = Join-Path $projectDir 'build\MapPoseHostCheck'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$exe = Join-Path $outputDir 'map_pose_host_test.exe'
& $compilerPath -std=c11 -Wall -Wextra -Werror `
    -I (Join-Path $PSScriptRoot 'map_include') -I (Join-Path $projectDir 'APP\Inc') `
    (Join-Path $projectDir 'APP\Src\map_pose.c') `
    (Join-Path $PSScriptRoot 'map_pose_host_test.c') -lm -o $exe
if ($LASTEXITCODE -ne 0) { throw "位姿测试编译失败：$LASTEXITCODE" }
& $exe
if ($LASTEXITCODE -ne 0) { throw "位姿测试失败：$LASTEXITCODE" }
