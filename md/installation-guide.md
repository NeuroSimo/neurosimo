# NeuroSimo installation guide

## Prerequisites

- Ubuntu 24.04.4 LTS installation media

## Table of contents

1. [Installing Ubuntu](#installing-ubuntu)
2. [Software installation](#software-installation)

## Installing Ubuntu

The following steps present one way to install Ubuntu 24.04.4 LTS on a computer with an empty SSD drive.

1. Boot Setup

   - Boot from Ubuntu 24.04.4 LTS installation media
   - Select "Try or Install Ubuntu"

2. Installation Configuration

   - Select "Minimal installation"
   - Enable:
     - Download updates while installing
     - Install third-party software for graphics and Wi-Fi hardware
   - Select "Something else" for installation type

3. Partition Setup

   - Create root partition:

     - Size: 150000 MB
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

## Software installation

After installing Ubuntu, run the following command to download and install NeuroSimo:

```bash
curl -fsSL https://raw.githubusercontent.com/neurosimo/neurosimo/main/scripts/install.sh | bash
```

This command will:
1. Install required dependencies including NVIDIA drivers
2. Clone the NeuroSimo repository to `~/neurosimo`
3. Run the installation script to install necessary software packages and dependencies

After the installation is complete, the script will prompt you to reboot the computer.

After reboot, NeuroSimo is ready to use. Launch it from your applications menu or run `neurosimo`.
