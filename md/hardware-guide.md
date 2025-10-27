# Hardware configuration guide

## Overview

NeuroSimo supports the following hardware devices:

- **EEG devices**: Bittium NeurOne and BrainProducts actiCHamp Plus (via TurboLink interface)
- **TMS triggering**: LabJack T4 (connected via USB)

## Table of contents

1. [EEG devices](#eeg-devices)
   - [Bittium NeurOne](#bittium-neurone)
   - [BrainProducts actiCHamp Plus (TurboLink)](#brainproducts-actichamp-plus-turbolink)
2. [TMS triggering](#tms-triggering)

## EEG devices

### Bittium NeurOne

Bittium NeurOne can be configured to send data via UDP to a specified port and IP address.
Configuring the real-time streaming consists of two steps: configuring the NeurOne and configuring the computer.

Here are the steps to configure the NeurOne:

1. Go to the protocol settings of the NeurOne and select "Digital Out" settings.
2. Set the Packet Frequency (Hz) equal to the sampling frequency of the EEG data.
3. Set "Target IP Address" to 192.168.200.254 and "Target Port" to 25000.
4. Enable "Send Triggers as Packets" and "Send Packets MeasurementStart and MeasurementEnd".
5. Click "Ok" to save the settings.

Here are the steps to configure the computer:

1. After connecting the EEG device to the computer via Ethernet cable, open the "Network" settings of the computer.
2. Select the Ethernet connection and click on "Properties".
3. Select the "IPv4" tab and set the "IP address" to 192.168.200.254 and "Subnet mask" to 255.255.255.0.
4. Click "OK" to save the settings.
5. Restart the computer.

To test the connection, start streaming the EEG data from the NeurOne, start a session in NeuroSimo, and check that
the EEG indicator is green in the top-right corner of the panel in the user interface.

### BrainProducts actiCHamp Plus (TurboLink)

TBD.

## TMS triggering

LabJack T4 is used to trigger the TMS device. In addition, an accessory called [LJTick DigitalOut5V](https://labjack.com/products/ljtick-digitalout5v) is needed for voltage conversion (3.3 V to 5 V) of the trigger signal.

In addition, a converter cable to convert from the wire ends to a BNC connector is convenient, so that one can use a
BNC cable to connect the LabJack outputs to the TMS device. See for example [this one](https://www.amazon.de/dp/B0B97D3WQJ)
on Amazon.

The LabJack outputs used are:

- FIO5: TMS trigger
- FIO4: Latency measurement trigger

Overall, the setup looks like this:

LabJack T4 -> LJTick DigitalOut5V (connected to FIO5, FIO4, and GND) -> Converter cable -> BNC cable -> TMS device

After LabJack T4 is connected to the computer via USB, it is automatically detected by NeuroSimo. It can be checked
by selecting 'example' decider in the user interface, setting up the EEG simulator to stream data, and pressing
'Start session'. It should send triggers to the TMS device every 2 seconds.
