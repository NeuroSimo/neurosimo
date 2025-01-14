# NeuroSimo installation guide

## Prerequisites

- Ubuntu 22.04.3 LTS installation media

## Table of contents

1. [Installing Real-time Ubuntu](#installing-real-time-ubuntu)
2. [Software installation](#software-installation)

## Installing Real-time Ubuntu

Ubuntu can be installed in various ways, but the following steps present one option for installing Ubuntu 22.04.3 LTS with
real-time kernel support on a computer with an empty SSD drive.

1. Boot Setup
   - Boot from Ubuntu 22.04.3 LTS installation media
   - Select "Try or Install Ubuntu"
   - If encountering Nouveau driver issues:
     ```
     Press 'e' at boot menu
     Change: linux /casper/vmlinuz ... quiet splash ---
     To: linux /casper/vmlinuz ... quiet splash nomodeset ---
     Press F10 or Ctrl+X to proceed
     ```

2. Installation Configuration
   - Select "Minimal installation"
   - Enable:
     - Download updates while installing
     - Install third-party software
   - Select "Something else" for installation type

3. Partition Setup
   - Create root partition:
     - Size: 120000 MB
     - Type: Primary
     - Location: Beginning
     - File system: Ext4
     - Mount point: /

   - Create home partition:
     - Size: Remaining space
     - Type: Primary
     - Location: Beginning
     - File system: Ext4
     - Mount point: /home

4. User Setup
   - Name: [Your Name]
   - Password: [Your Password]
   - Enable "Require password to log in"

### Ubuntu configuration

Ubuntu Pro can be configured to enable real-time kernel support. The following steps provide a guide for setting up Ubuntu Pro on a new computer.

1. Install Ubuntu Pro
   - Complete initial Ubuntu Pro setup
   - Create personal Ubuntu Pro account

2. Install Real-time Kernel
   ```bash
   sudo pro enable realtime-kernel
   ```
## Software installation

After installing Ubuntu, clone the NeuroSimo repository to the home directory:

```bash
cd ~
git clone https://github.com/neurosimo/neurosimo --recurse-submodules
```

Install Ansible:

```bash
sudo apt update && sudo apt install pipx
pipx install --include-deps ansible
```

Navigate to the repository directory and run the installation script:

```bash
cd neurosimo
scripts/install-neurosimo
```

The script will run the Ansible playbook to install the necessary software packages and dependencies for NeuroSimo.

After that, build the NeuroSimo project:

```bash
scripts/build-neurosimo
```

Modify `.env` file in the repository root to set the environment, including the EEG and
TMS device settings.

Finally, reboot the computer.

After reboot, NeuroSimo should be installed and ready to use via the web interface at
[http://localhost:3000](http://localhost:3000).

## Web UI setup
To create a desktop link to the NeuroSimo panel:
   - Open Chrome and navigate to [http://localhost:3000](http://localhost:3000).
   - Click "Install NeuroSimo panel"
   - Enable launching for desktop shortcut
