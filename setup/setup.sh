#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "Starting dependency installation..."

if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    sudo apt-get update
    sudo apt-get install -y build-essential cmake ninja-build python3 python3-pip git golang-go
elif [[ "$OSTYPE" == "darwin"* ]]; then
    if ! command -v brew >/dev/null 2>&1; then
        echo "Homebrew not found. Please install it at https://brew.sh/"
        exit 1
    fi
    brew install cmake ninja python git go
else
    echo "Unsupported OS for setup/setup.sh. Use setup/setup.ps1 on Windows or install dependencies manually."
    exit 1
fi

python3 -m pip install --upgrade pip
python3 -m pip install textual

if command -v go >/dev/null 2>&1; then
    (cd "${ROOT_DIR}" && go mod download)
fi

echo "Success! You can now build the project:"
echo "cmake -B build -G \"Unix Makefiles\" && cmake --build build"
echo "Run the Python TUI with: python3 build_tui.py"
echo "Run the Go TUI with: go run TUI.go"
