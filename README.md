# NeuroSimo

NeuroSimo is an open-source software platform for real-time EEG- and EMG-guided transcranial magnetic stimulation (TMS) [1].

## Installation

Run this command to install NeuroSimo:

```bash
curl -fsSL https://raw.githubusercontent.com/neurosimo/neurosimo/main/scripts/install.sh | bash
```

This will install NeuroSimo to `~/neurosimo`.

If you're new to Linux, see the [Installation guide](md/installation-guide.md) for detailed instructions.

## Hardware configuration

NeuroSimo currently supports two EEG devices: Bittium NeurOne and BrainProducts actiCHamp Plus (via TurboLink interface).

Triggering the TMS device is done via LabJack T4, which is connected to the computer via USB.

See the [Hardware configuration guide](md/hardware-guide.md) for instructions on how to set up the EEG and TMS devices.

## Getting started

See the [Getting started guide](md/getting-started.md) for instructions on how to use NeuroSimo.

## License

This project is licensed under the GPL v3 License - see the [LICENSE](LICENSE) file for details.

## References

1. Kahilakoski, O.-P., Alkio, K., Siljamo, O., Valén, K., Laurinoja, J., Haxel, L., Makkonen, M., Mutanen, T. P., Tommila, T., Guidotti, R., Pieramico, G., Ilmoniemi, R. J., & Roine, T. (2025). **NeuroSimo: an open-source software for real-time EEG- and EMG-guided TMS**. [bioRxiv](https://www.biorxiv.org/content/10.1101/2025.04.05.647342v1)
