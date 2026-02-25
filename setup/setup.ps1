$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RootDir = Resolve-Path (Join-Path $ScriptDir "..")

Write-Host "Starting dependency installation..."

if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
    Write-Error "winget not found. Install CMake, Ninja, Python 3, Git, and Go manually."
}

winget install --id Kitware.CMake -e --accept-source-agreements --accept-package-agreements
winget install --id Ninja-build.Ninja -e --accept-source-agreements --accept-package-agreements
winget install --id Python.Python.3.12 -e --accept-source-agreements --accept-package-agreements
winget install --id Git.Git -e --accept-source-agreements --accept-package-agreements
winget install --id GoLang.Go -e --accept-source-agreements --accept-package-agreements

python -m pip install --upgrade pip
python -m pip install textual

if (Get-Command go -ErrorAction SilentlyContinue) {
    Push-Location $RootDir
    go mod download
    Pop-Location
}

Write-Host "Success! You can now build the project:"
Write-Host "cmake -B build -G \"NMake Makefiles\""
Write-Host "cmake --build build"
Write-Host "Run the Python TUI with: python build_tui.py"
Write-Host "Run the Go TUI with: go run TUI.go"
