"""
magventure_tms.py

Minimal direct-serial control module for MagVenture MagPro devices.

Purpose
-------
Configure the MagPro for externally triggered stimulation via the BNC trigger
input:

    set_single_pulse(amplitude)
        One external BNC trigger -> one ordinary Biphasic pulse.

    set_tbs(amplitude, burst_pulses=3, ipi_ms=20)
        One external BNC trigger -> one burst:
        Biphasic Burst, configurable pulses per burst and intra-burst IPI.

Default TBS-like settings are:
    burst_pulses = 3
    ipi_ms = 20

which correspond to a conventional theta-burst element:
    3 pulses at 50 Hz intra-burst frequency.

This module does not fire pulses, start trains, generate timing, or switch the
visible page/tab on the MagPro screen. It only sets the device configuration and
verifies the resulting state by reading it back from the stimulator.

Observed devices
----------------
This direct-serial implementation was tested against:

    - MagVenture MagPro R30
    - MagVenture MagPro X100+Option

Both accepted the same direct configuration command sequence for the two modes
above. Both devices applied the configuration without sending a direct
acknowledgement frame for the configuration-write command.

R30-specific readback note
--------------------------
On the R30, get_status() reports the model cleanly as "R30". However, the
get_settings() response contains a byte that this parser does not interpret as
a standard model code; in debug versions this may appear as something like
"unknown(88)". This module deliberately does not use the model field from
get_settings() for validation. It uses get_status() for device identity and
get_settings() only for the fields that matter for burst validation:

    - Waveform
    - burstPulses
    - ipivalue
    - currentDirection / BA_Ratio, when useful for diagnostics

Why not MAGICpy?
----------------
MAGICpy is a useful Python translation of the MagVenture portion of the MATLAB
MAGIC toolbox. In our tests, however, MAGICpy reported failures for configuration
commands such as set_waveform(), set_burst(), and set_ipi(), typically returning
values like:

    ("Empty", 1)
    ("Too short", 1)

Despite those reported failures, the MagPro front panel and subsequent readback
showed that the settings had actually changed.

The direct serial tests clarified the likely reason: the MagVenture configuration
write command used here does not send a useful write acknowledgement. It is a
no-ACK write. MAGICpy appears to treat the missing/short write response as a
command failure, even though the device applies the command.

Therefore this module uses the more robust pattern:

    1. Send the configuration write command.
    2. Do not expect a direct acknowledgement for that write.
    3. Wait briefly for the device to settle.
    4. Query status/settings.
    5. Raise an exception unless readback confirms the requested state.

This makes the final success criterion explicit readback verification, not the
return value of the write command.

Frame-synchronized serial reading
---------------------------------
When the stimulator is armed or has just delivered a pulse, the serial stream
may contain stale or asynchronous frames. A fixed-length read can accidentally
combine the end of one frame with the start of the next, for example:

    FE 04 02 00 00 1C 39 FF FE 09 00 1C 00 ...

The first part is a complete 8-byte frame, followed by the start of a status
frame. To avoid this, this module reads complete FE...FF frames and ignores
frames whose command byte is not the response currently requested.

Timing note
-----------
The default config_settle_s=0.30 is intentional. The configuration command has
no acknowledgement, and immediately sending another command can occasionally
miss the next response, especially in Docker/USB-serial setups. The settle delay
is small relative to experimental flow and does not affect pulse timing, because
actual stimulation timing is controlled by the external BNC trigger.

Screen/page note
----------------
Earlier test scripts called a "set Train page" command before configuring TBS.
That command switched the visible tab/page on the MagPro screen. It is not used
here. The direct configuration command is sufficient for external-triggered
single-pulse/burst switching, and readback verification confirms the actual
mode/settings. The displayed waveform may still update because the real device
configuration changes, but this module does not intentionally switch the screen
page/tab.

Docker note
-----------
The serial device must be passed through to the container, for example:

    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0

The container user also needs permission to access the device, usually via the
host's dialout group or an equivalent numeric group_add entry.

Dependency
----------
    pip install pyserial

Example
-------
    from magventure_tms import MagVentureTMS

    with MagVentureTMS("/dev/ttyUSB0") as tms:
        tms.set_single_pulse(50)
        # external BNC trigger -> one ordinary Biphasic pulse

        tms.set_tbs(50)
        # external BNC trigger -> one 3-pulse burst, 20 ms IPI

        tms.set_tbs(50, burst_pulses=2, ipi_ms=30)
        # external BNC trigger -> one 2-pulse burst, 30 ms IPI
"""

from __future__ import annotations

import time
from typing import Any, Optional

import serial


class MagVentureError(RuntimeError):
    """Raised when communication or readback validation fails."""


MODEL = {
    0: "R30",
    1: "X100",
    2: "R30+Option",
    3: "X100+Option",
    4: "R30+Option+Mono",
    5: "MST",
}

BYTE_TO_MODE = {
    0x00: "Standard",
    0x01: "Power",
    0x02: "Twin",
    0x03: "Dual",
}

BYTE_TO_WAVEFORM = {
    0x00: "Monophasic",
    0x01: "Biphasic",
    0x02: "Half Sine",
    0x03: "Biphasic Burst",
}

BYTE_TO_CURRENT_DIR = {
    0x00: "Normal",
    0x01: "Reverse",
}

MODE_STANDARD = 0x00
CURRENT_NORMAL = 0x00
WAVEFORM_BIPHASIC = 0x01
WAVEFORM_BIPHASIC_BURST = 0x03

# MagVenture/MAGIC encodes burst count in reverse order.
BURST_TO_BYTE = {2: 0x03, 3: 0x02, 4: 0x01, 5: 0x00}
BYTE_TO_BURST = {0x03: 2, 0x02: 3, 0x01: 4, 0x00: 5}


def _crc8_maxim(command_bytes: bytes) -> int:
    """
    CRC-8 Dallas/Maxim over command bytes only.

    The direct serial frame is:

        FE <length> <command bytes...> <crc> FF

    The CRC is computed over the command bytes, not over the FE/length/FF bytes.
    """
    crc = 0
    for byte in command_bytes:
        cur = byte
        for _ in range(8):
            mix = (crc ^ cur) & 0x01
            crc >>= 1
            if mix:
                crc ^= 0x8C
            cur >>= 1
    return crc & 0xFF


def _u16_be(value: int) -> tuple[int, int]:
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"value out of uint16 range: {value}")
    return (value >> 8) & 0xFF, value & 0xFF


def _hex(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def _parse_status(data: bytes) -> dict[str, Any]:
    if len(data) != 13:
        raise MagVentureError(f"expected 13 status bytes, got {len(data)}: {_hex(data)}")
    if data[0] != 0xFE or data[-1] != 0xFF or data[2] != 0x00:
        raise MagVentureError(f"bad status frame: {_hex(data)}")

    combined = data[3]
    model_bits = (combined >> 5) & 0b111
    enabled = (combined >> 4) & 0b1
    waveform_bits = (combined >> 2) & 0b11
    mode_bits = combined & 0b11

    return {
        "Mode": BYTE_TO_MODE.get(mode_bits, f"unknown({mode_bits})"),
        "Waveform": BYTE_TO_WAVEFORM.get(waveform_bits, f"unknown({waveform_bits})"),
        "Status": "Enabled" if enabled else "Disabled",
        "Model": MODEL.get(model_bits, f"unknown({model_bits})"),
        "SerialNo": (data[4] << 16) | (data[5] << 8) | data[6],
        "Temperature": data[7],
        "coilTypeNo": data[8],
        "amplitudePercentage_A": data[9],
        "amplitudePercentage_B": data[10],
        "_raw_model": model_bits,
        "_raw": _hex(data),
    }


def _parse_settings(data: bytes) -> dict[str, Any]:
    if len(data) != 14:
        raise MagVentureError(f"expected 14 settings bytes, got {len(data)}: {_hex(data)}")
    if data[0] != 0xFE or data[-1] != 0xFF or data[2] != 0x09:
        raise MagVentureError(f"bad settings frame: {_hex(data)}")

    mode_b = data[5]
    current_b = data[6]
    waveform_b = data[7]
    burst_b = data[8]
    ipi_index = (data[10] << 8) | data[9]
    ba_index = data[11]

    waveform = BYTE_TO_WAVEFORM.get(waveform_b, f"unknown({waveform_b})")

    ipi_value: Optional[float] = None
    if waveform == "Biphasic Burst":
        # This mirrors the MAGIC/MagVenture mapping for Biphasic Burst IPI.
        if 0 <= ipi_index <= 80:
            ipi_value = 100 - ipi_index
        elif 80 < ipi_index <= 100:
            ipi_value = 20 - (ipi_index - 80) * 0.5
        elif 100 < ipi_index <= 195:
            ipi_value = 10 - (ipi_index - 100) * 0.1

    return {
        "Mode": BYTE_TO_MODE.get(mode_b, f"unknown({mode_b})"),
        "currentDirection": BYTE_TO_CURRENT_DIR.get(current_b, f"unknown({current_b})"),
        "Waveform": waveform,
        "burstPulses": BYTE_TO_BURST.get(burst_b, f"unknown({burst_b})"),
        "ipivalue": ipi_value,
        "BA_Ratio": 5 - ba_index * 0.05,
        "_ipi_index_raw": ipi_index,
        "_raw": _hex(data),
    }


class MagVentureTMS:
    """
    Minimal MagVenture MagPro controller for external-trigger mode switching.

    Public methods:
        get_status()
        get_settings()
        set_single_pulse(amplitude)
        set_tbs(amplitude, burst_pulses=3, ipi_ms=20)
        close()

    The mode-setting methods raise MagVentureError if readback does not confirm
    the requested state.
    """

    def __init__(
        self,
        usb_device: str = "/dev/ttyUSB0",
        *,
        timeout: float = 1.0,
        config_settle_s: float = 0.30,
    ) -> None:
        """
        Parameters
        ----------
        usb_device:
            Serial device path, e.g. "/dev/ttyUSB0".

        timeout:
            Serial read/write timeout. The default is deliberately conservative
            for Docker + USB-serial use. It does not add a full second when the
            device replies promptly.

        config_settle_s:
            Delay after the no-ACK configuration write before the next command.
            This avoids missing the next response on Docker/USB-serial systems.
        """
        self.usb_device = usb_device
        self.timeout = timeout
        self.config_settle_s = config_settle_s
        try:
            self._ser = serial.Serial(
                usb_device,
                baudrate=38400,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout,
                write_timeout=timeout,
            )
        except serial.SerialException as exc:
            raise MagVentureError(
                f"Cannot open MagVenture serial port {usb_device!r}. "
                "Check that the USB-serial adapter is connected and the device path is correct."
            ) from exc
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()

    def __enter__(self) -> "MagVentureTMS":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def get_status(self) -> dict[str, Any]:
        """
        Return basic device status, including model, waveform, enable status,
        serial number, temperature, coil type, and amplitudes.
        """
        response = self._send_and_read_response([0x00], expected_command=0x00)
        return _parse_status(response)

    def get_settings(self) -> dict[str, Any]:
        """
        Return detailed settings, including waveform, burst pulse count, and IPI.

        On R30, do not use the get_settings model byte as a device identity
        source. Use get_status()["Model"] for that.
        """
        response = self._send_and_read_response(
            [0x09, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            expected_command=0x09,
        )
        return _parse_settings(response)

    def set_single_pulse(self, amplitude: int) -> dict[str, Any]:
        """
        Configure external-trigger single-pulse mode.

        After this succeeds:
            one external BNC trigger should produce one ordinary Biphasic pulse.

        Returns
        -------
        dict
            Verified status dictionary.
        """
        self._validate_amplitude(amplitude)

        status_before = self.get_status()
        self._write_config(
            model_byte=status_before["_raw_model"],
            waveform_byte=WAVEFORM_BIPHASIC,
            burst_pulses=3,
            ipi_ms=20,
        )
        self._set_amplitude(amplitude)

        status = self.get_status()
        self._require(status["Waveform"] == "Biphasic", f"expected Biphasic, got {status['Waveform']!r}")
        self._require(
            status["amplitudePercentage_A"] == amplitude,
            f"expected amplitude {amplitude}, got {status['amplitudePercentage_A']}",
        )
        return status

    def set_tbs(
        self,
        amplitude: int,
        *,
        burst_pulses: int = 3,
        ipi_ms: float = 20,
    ) -> dict[str, Any]:
        """
        Configure external-trigger burst/TBS mode.

        After this succeeds:
            one external BNC trigger should produce one burst with the requested
            number of pulses and intra-burst IPI.

        Defaults:
            burst_pulses=3
            ipi_ms=20

        These defaults correspond to a conventional theta-burst element:
            3 pulses at 50 Hz within-burst timing.

        This is a single-burst setting. It does not configure an iTBS train.
        If iTBS timing is needed, the external trigger source should send one
        trigger per burst, e.g. 5 Hz during the active period.

        Returns
        -------
        dict
            {"status": ..., "settings": ...}
        """
        self._validate_amplitude(amplitude)
        self._validate_burst_pulses(burst_pulses)
        self._validate_ipi_ms(ipi_ms)

        status_before = self.get_status()

        # Intentionally no set-page command here. We do not switch the visible
        # tab/page on the MagPro screen.
        self._write_config(
            model_byte=status_before["_raw_model"],
            waveform_byte=WAVEFORM_BIPHASIC_BURST,
            burst_pulses=burst_pulses,
            ipi_ms=ipi_ms,
        )
        self._set_amplitude(amplitude)

        status = self.get_status()
        settings = self.get_settings()

        self._require(
            status["Waveform"] == "Biphasic Burst",
            f"expected status waveform Biphasic Burst, got {status['Waveform']!r}",
        )
        self._require(
            settings["Waveform"] == "Biphasic Burst",
            f"expected settings waveform Biphasic Burst, got {settings['Waveform']!r}",
        )
        self._require(
            settings["burstPulses"] == burst_pulses,
            f"expected {burst_pulses} burst pulses, got {settings['burstPulses']!r}",
        )
        self._require(
            settings["ipivalue"] is not None and abs(float(settings["ipivalue"]) - float(ipi_ms)) < 0.01,
            f"expected IPI {ipi_ms} ms, got {settings['ipivalue']!r}",
        )
        self._require(
            status["amplitudePercentage_A"] == amplitude,
            f"expected amplitude {amplitude}, got {status['amplitudePercentage_A']}",
        )

        return {"status": status, "settings": settings}

    def _frame(self, command_bytes: list[int]) -> bytes:
        body = bytes(command_bytes)
        return bytes([0xFE, len(body)]) + body + bytes([_crc8_maxim(body), 0xFF])

    def _send_no_response(self, command_bytes: list[int]) -> None:
        self._ser.write(self._frame(command_bytes))
        self._ser.flush()

    def _send_and_read_response(self, command_bytes: list[int], *, expected_command: int) -> bytes:
        self._ser.write(self._frame(command_bytes))
        self._ser.flush()
        return self._read_matching_frame(expected_command=expected_command)

    def _read_matching_frame(self, *, expected_command: int) -> bytes:
        """
        Read complete FE...FF frames until one has the requested command byte.

        This prevents stale/asynchronous frames from corrupting the next parsed
        response. Example failure this avoids:

            FE 04 02 00 00 1C 39 FF FE 09 00 1C 00 ...

        where a stale 8-byte frame is followed by the beginning of a status
        frame. A fixed 13-byte read would incorrectly combine them.
        """
        deadline = time.monotonic() + self.timeout

        last_frame: Optional[bytes] = None
        while time.monotonic() < deadline:
            frame = self._read_one_frame(deadline)
            if frame is None:
                continue

            last_frame = frame
            if len(frame) >= 3 and frame[2] == expected_command:
                return frame

            # Otherwise ignore unrelated/stale frame and keep looking.

        if last_frame is not None:
            raise MagVentureError(
                f"timed out waiting for response command 0x{expected_command:02X}; "
                f"last unrelated frame: {_hex(last_frame)}"
            )
        raise MagVentureError(f"timed out waiting for response command 0x{expected_command:02X}")

    def _read_one_frame(self, deadline: float) -> Optional[bytes]:
        """
        Read one complete frame:
            FE <length> <length command bytes> <crc> FF

        Returns None if no start byte is seen before the deadline.
        """
        # Find FE.
        while time.monotonic() < deadline:
            b = self._ser.read(1)
            if not b:
                continue
            if b == b"\xFE":
                break
        else:
            return None

        length_b = self._ser.read(1)
        if not length_b:
            return None

        length = length_b[0]
        rest = self._read_exact(length + 2, deadline)
        if rest is None:
            return None

        frame = b"\xFE" + length_b + rest
        if frame[-1] != 0xFF:
            # Bad framing. Return it so the eventual error can show what happened.
            return frame

        return frame

    def _read_exact(self, n: int, deadline: float) -> Optional[bytes]:
        chunks = bytearray()
        while len(chunks) < n and time.monotonic() < deadline:
            part = self._ser.read(n - len(chunks))
            if part:
                chunks.extend(part)
        if len(chunks) != n:
            return None
        return bytes(chunks)

    def _set_amplitude(self, amplitude: int) -> None:
        response = self._send_and_read_response([0x01, amplitude], expected_command=0x01)
        if response[0] != 0xFE or response[-1] != 0xFF:
            raise MagVentureError(f"bad amplitude response: {_hex(response)}")

    def _write_config(
        self,
        *,
        model_byte: int,
        waveform_byte: int,
        burst_pulses: int,
        ipi_ms: float,
    ) -> None:
        """
        Send the no-ACK configuration-write command.

        The device is expected not to reply directly to this write. The caller
        verifies success through subsequent status/settings reads.
        """
        ipi_m, ipi_l = _u16_be(int(round(float(ipi_ms) * 10)))
        ba_m, ba_l = _u16_be(100)  # B/A ratio = 1.0

        command = [
            0x09,
            0x01,
            model_byte,
            MODE_STANDARD,
            CURRENT_NORMAL,
            waveform_byte,
            BURST_TO_BYTE[burst_pulses],
            ipi_m,
            ipi_l,
            ba_m,
            ba_l,
        ]
        self._send_no_response(command)
        time.sleep(self.config_settle_s)

    @staticmethod
    def _validate_amplitude(amplitude: int) -> None:
        if not isinstance(amplitude, int):
            raise TypeError("amplitude must be an integer")
        if not 0 <= amplitude <= 100:
            raise ValueError("amplitude must be in range 0..100")

    @staticmethod
    def _validate_burst_pulses(burst_pulses: int) -> None:
        if burst_pulses not in BURST_TO_BYTE:
            raise ValueError("burst_pulses must be one of 2, 3, 4, or 5")

    @staticmethod
    def _validate_ipi_ms(ipi_ms: float) -> None:
        if not isinstance(ipi_ms, (int, float)):
            raise TypeError("ipi_ms must be numeric")
        if ipi_ms <= 0:
            raise ValueError("ipi_ms must be positive")

    @staticmethod
    def _require(condition: bool, message: str) -> None:
        if not condition:
            raise MagVentureError(message)
