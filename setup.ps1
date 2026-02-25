$ErrorActionPreference = "Stop"

Write-Host "Starting dependency installation..."

if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
    Write-Error "winget not found. Install CMake, Ninja, Python 3, and Git manually."
}

winget install --id Kitware.CMake -e --accept-source-agreements --accept-package-agreements
winget install --id Ninja-build.Ninja -e --accept-source-agreements --accept-package-agreements
winget install --id Python.Python.3.12 -e --accept-source-agreements --accept-package-agreements
winget install --id Git.Git -e --accept-source-agreements --accept-package-agreements

python -m pip install --upgrade pip
python -m pip install textual

Write-Host "Success! You can now build the project:"
Write-Host "cmake -B build -G \"NMake Makefiles\""
Write-Host "cmake --build build"
Write-Host "Run the interactive TUI with: python build_tui.py"