#!/bin/bash
# setup.sh - Downloads external dependencies

mkdir -p external
cd external

# Clone Breach (Replace URL with the exact fork/repo you use if different)
echo "Cloning Breach..."
git clone https://github.com/decyphir/breach.git

echo "Setup complete. Breach is ready to use."