from __future__ import annotations

import time
from typing import Any


class MockTMS:
    """
    Mock TMS controller with the same public API shape as MagVentureTMS.

    It does not simulate pulses or serial communication. It only simulates
    mode switching, where each actual mode change takes switch_delay_s seconds.
    """

    MODE_PRIME_TRIPLET = "prime_triplet"
    MODE_PRIME_SINGLE_PULSE = "prime_single_pulse"
    MODE_PREDETERMINED = "predetermined"

    def __init__(
        self,
        usb_device: str = "/dev/mock-tms",
        *,
        timeout: float = 1.0,
        config_settle_s: float = 0.30,
        switch_delay_s: float = 0.8,
        initial_mode: str = MODE_PRIME_SINGLE_PULSE,
    ) -> None:
        # Keep these constructor args for API compatibility with MagVentureTMS,
        # even though the mock does not use them.
        self.usb_device = usb_device
        self.timeout = timeout
        self.config_settle_s = config_settle_s

        self.switch_delay_s = switch_delay_s
        self.mode = initial_mode
        self.closed = False
        self.amplitude: int | None = None

    def close(self) -> None:
        self.closed = True

    def __enter__(self) -> "MockTMS":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def get_status(self) -> dict[str, Any]:
        self._require_open()

        return {
            "Mode": "Standard",
            "Waveform": self._waveform(),
            "Status": "Enabled",
            "Model": "MockTMS",
            "SerialNo": 0,
            "Temperature": 0,
            "coilTypeNo": 0,
            "amplitudePercentage_A": self.amplitude,
            "amplitudePercentage_B": 0,
            "mode": self.mode,
        }

    def get_settings(self) -> dict[str, Any]:
        self._require_open()

        return {
            "Mode": "Standard",
            "currentDirection": "Normal",
            "Waveform": self._waveform(),
            "burstPulses": 3 if self.mode == self.MODE_PRIME_TRIPLET else None,
            "ipivalue": 20 if self.mode == self.MODE_PRIME_TRIPLET else None,
            "BA_Ratio": 1.0,
            "mode": self.mode,
        }

    def set_single_pulse(self, amplitude: int) -> dict[str, Any]:
        """
        API-compatible mock of MagVentureTMS.set_single_pulse().
        """
        self._validate_amplitude(amplitude)
        self._set_mode(self.MODE_PRIME_SINGLE_PULSE)
        self.amplitude = amplitude
        return self.get_status()

    def set_tbs(
        self,
        amplitude: int,
        *,
        burst_pulses: int = 3,
        ipi_ms: float = 20,
    ) -> dict[str, Any]:
        """
        API-compatible mock of MagVentureTMS.set_tbs().

        The mock accepts the same arguments but only uses them for lightweight
        validation/status reporting.
        """
        self._validate_amplitude(amplitude)
        self._set_mode(self.MODE_PRIME_TRIPLET)
        self.amplitude = amplitude

        status = self.get_status()
        settings = self.get_settings()
        settings["burstPulses"] = burst_pulses
        settings["ipivalue"] = ipi_ms

        return {"status": status, "settings": settings}

    def set_predetermined(self, amplitude: int) -> dict[str, Any]:
        """
        Predetermined is represented as its own mode but behaves as single pulse.
        """
        self._validate_amplitude(amplitude)
        self._set_mode(self.MODE_PREDETERMINED)
        self.amplitude = amplitude
        return self.get_status()

    def _set_mode(self, mode: str) -> None:
        self._require_open()

        time.sleep(self.switch_delay_s)
        self.mode = mode

    def _waveform(self) -> str:
        if self.mode == self.MODE_PRIME_TRIPLET:
            return "Biphasic Burst"
        return "Biphasic"

    def _require_open(self) -> None:
        if self.closed:
            raise RuntimeError("MockTMS is closed")

    @staticmethod
    def _validate_amplitude(amplitude: int) -> None:
        if not isinstance(amplitude, int):
            raise TypeError("amplitude must be an integer")
        if not 0 <= amplitude <= 100:
            raise ValueError("amplitude must be in range 0..100")