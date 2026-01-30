# NeuroSimo

NeuroSimo is an open-source software platform for real-time EEG- and EMG-guided transcranial magnetic stimulation (TMS) [1].

**Note:** mTMS integration was maintained until v0.2.0 only. If you need mTMS support, please use v0.2.0.

## Installation

Run this command to install NeuroSimo:

```bash
curl -fsSL https://raw.githubusercontent.com/neurosimo/neurosimo/main/scripts/install.sh | bash
```

This will install NeuroSimo to `~/neurosimo`.

For more detailed instructions, see the [Installation guide](md/installation-guide.md).

## Hardware configuration

NeuroSimo currently supports the following EEG devices: Bittium NeurOne and BrainProducts actiCHamp Plus (via TurboLink interface).

Triggering the TMS device is done via LabJack T4, connected to the computer via USB.

See the [Hardware configuration guide](md/hardware-guide.md) for instructions on how to set up the EEG and TMS devices.

## Getting started

See the [Getting started guide](md/getting-started.md) for instructions on how to use NeuroSimo.

## Changelog

See [CHANGELOG.md](CHANGELOG.md) for a list of changes in each release.

## License

This project is licensed under the GPL v3 License - see the [LICENSE](LICENSE) file for details.

## References

1. Kahilakoski, O.-P., Alkio, K., Siljamo, O., Valén, K., Laurinoja, J., Haxel, L., Makkonen, M., Mutanen, T. P., Tommila, T., Guidotti, R., Pieramico, G., Ilmoniemi, R. J., & Roine, T. (2025). **NeuroSimo: an open-source software for real-time EEG- and EMG-guided TMS**. [bioRxiv](https://www.biorxiv.org/content/10.1101/2025.04.05.647342v1)
