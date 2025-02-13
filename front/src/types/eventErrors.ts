export const ChargeError = {
  NO_ERROR: 0,
  INVALID_EXECUTION_CONDITION: 1,
  INVALID_CHANNEL: 2,
  INVALID_VOLTAGE: 3,
  LATE: 4,
  OVERLAPPING_WITH_DISCHARGING: 5,
  OVERLAPPING_WITH_STIMULATION: 6,
  CHARGING_FAILURE: 7,
  UNKNOWN_ERROR: 8,
}

export const PulseError = {
  NO_ERROR: 0,
  INVALID_EXECUTION_CONDITION: 1,
  INVALID_CHANNEL: 2,
  INVALID_NUMBER_OF_WAVEFORM_PIECES: 3,
  INVALID_MODES: 4,
  INVALID_DURATIONS: 5,
  LATE: 6,
  TOO_MANY_PULSES: 7,
  OVERLAPPING_WITH_CHARGING: 8,
  OVERLAPPING_WITH_DISCHARGING: 9,
  NOT_ALLOWED: 10,
  TRIGGERING_FAILURE: 11,
  UNKNOWN_ERROR: 12,
}

export const TriggerOutError = {
  NO_ERROR: 0,
  INVALID_EXECUTION_CONDITION: 1,
  LATE: 2,
  TRIGGEROUT_FAILURE: 3,
  UNKNOWN_ERROR: 4,
}

export const DischargeError = {
  NO_ERROR: 0,
  INVALID_EXECUTION_CONDITION: 1,
  INVALID_CHANNEL: 2,
  INVALID_VOLTAGE: 3,
  LATE: 4,
  OVERLAPPING_WITH_CHARGING: 5,
  OVERLAPPING_WITH_STIMULATION: 6,
  DISCHARGING_FAILURE: 7,
  UNKNOWN_ERROR: 8,
}

export const errorsByType = {
  pulse: PulseError,
  triggerOut: TriggerOutError,
  charge: ChargeError,
  discharge: DischargeError,
}
