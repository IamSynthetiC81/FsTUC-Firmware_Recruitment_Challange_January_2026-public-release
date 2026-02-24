$ErrorActionPreference = "Stop"

Write-Host "Starting dependency installation..."

if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
    Write-Error "winget not found. Install CMake, Ninja, Python 3, and Git manually."
}

winget install --id Kitware.CMake -e --accept-source-agreements --accept-package-agreements
winget install --id Ninja-build.Ninja -e --accept-source-agreements --accept-package-agreements
winget install --id Python.Python.3.12 -e --accept-source-agreements --accept-package-agreements
winget install --id Git.Git -e --accept-source-agreements --accept-package-agreements

Write-Host "Success! You can now build the project:"
Write-Host "cmake -B build -G \"Unix Makefiles\""
Write-Host "cmake --build build"