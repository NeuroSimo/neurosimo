#!/bin/bash
set -e

INSTALL_DIR="$HOME/neurosimo"

echo "Installing Neurosimo to: $INSTALL_DIR"

# Install dependencies
sudo apt update
sudo apt install -y git

# Check if already exists
if [ -d "$INSTALL_DIR" ]; then
    echo ""
    echo "ERROR: Directory $INSTALL_DIR already exists."
    echo ""
    echo "If you want to reinstall, please remove the existing directory first:"
    echo "  rm -rf $INSTALL_DIR"
    echo ""
    echo "Then run this installation script again."
    echo ""
    exit 1
fi

# Clone to the canonical location
git clone https://github.com/neurosimo/neurosimo --recurse-submodules "$INSTALL_DIR"

# Run installer
cd "$INSTALL_DIR"
./scripts/install-neurosimo

echo ""
echo "For first-time installations, a reboot is required for all changes to take effect"
echo ""
read -p "Do you want to reboot now? [y/N] " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Rebooting system..."
    sudo reboot
else
    echo "Reboot skipped."
fi
echo ""
