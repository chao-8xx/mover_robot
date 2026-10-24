param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectDir = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$compilerPath = (Get-Command $Compiler -ErrorAction Stop).Source
# MinGW的编译子进程需要能找到同目录DLL。
$env:PATH = (Split-Path -Parent $compilerPath) + ';' + $env:PATH
$outputDir = Join-Path $projectDir 'build\CameraHostCheck'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$exe = Join-Path $outputDir 'camera_host_test.exe'
& $compilerPath -std=c11 -Wall -Wextra -Werror `
    -I (Join-Path $PSScriptRoot 'include') -I (Join-Path $projectDir 'BSP\Inc') `
    (Join-Path $projectDir 'BSP\Src\camera.c') (Join-Path $PSScriptRoot 'camera_host_test.c') `
    -o $exe
if ($LASTEXITCODE -ne 0) { throw "camera测试编译失败：$LASTEXITCODE" }
& $exe
if ($LASTEXITCODE -ne 0) { throw "camera测试失败：$LASTEXITCODE" }
