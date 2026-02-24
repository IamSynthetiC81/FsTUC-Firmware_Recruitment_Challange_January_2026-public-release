#!/usr/bin/env bash
set -euo pipefail

echo "Starting dependency installation..."

if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    sudo apt-get update
    sudo apt-get install -y build-essential cmake ninja-build python3 python3-pip git
elif [[ "$OSTYPE" == "darwin"* ]]; then
    if ! command -v brew >/dev/null 2>&1; then
        echo "Homebrew not found. Please install it at https://brew.sh/"
        exit 1
    fi
    brew install cmake ninja python git
else
    echo "Unsupported OS for setup.sh. Use setup.ps1 on Windows or install dependencies manually."
    exit 1
fi

python3 -m pip install --upgrade pip

echo "Success! You can now build the project:"
echo "cmake -B build -G \"Unix Makefiles\" && cmake --build build"
