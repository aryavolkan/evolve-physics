#!/bin/bash
# Build and install the Python module

set -e

echo "Building evolve-physics Python module..."

# Check if maturin is installed
if ! command -v maturin &> /dev/null; then
    echo "Installing maturin..."
    pip install maturin
fi

# Build the wheel
echo "Building wheel..."
maturin build --release

# Find the wheel file
WHEEL=$(ls -t target/wheels/*.whl | head -1)

if [ -z "$WHEEL" ]; then
    echo "Error: No wheel file found!"
    exit 1
fi

echo "Built wheel: $WHEEL"

# Install the wheel
echo "Installing wheel..."
pip install --force-reinstall "$WHEEL"

echo "Installation complete!"
echo ""
echo "Test with:"
echo "  python -c 'import evolve_physics; print(evolve_physics.__version__)'"
echo "  python examples/evolve_training.py"