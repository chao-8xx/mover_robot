$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
$runtime = Get-Command python -ErrorAction SilentlyContinue
if ($runtime) {
    & $runtime.Source (Join-Path $PSScriptRoot 'server.py')
} elseif (Test-Path -LiteralPath 'D:\AnaConda\python.exe') {
    & 'D:\AnaConda\python.exe' (Join-Path $PSScriptRoot 'server.py')
} else {
    Write-Host 'Python 3.10+ is required. Install Python, then run python server.py.'
    Read-Host 'Press Enter to close'
    exit 1
}
