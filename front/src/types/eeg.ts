import ROSLIB from 'roslib'

export const EventTypes = {
  CHARGE: 0,
  PULSE: 1,
  DISCHARGE: 2,
  TRIGGER_OUT: 3,
}

/* TODO: Not up-to-date. */
export interface Sample {
  eeg_data: number[]
  emg_data: number[]
  time: number
}

export interface SampleMetadata extends ROSLIB.Message {
  sampling_frequency: number
  num_of_eeg_channels: number
  num_of_emg_channels: number
}

export interface SampleMessage extends ROSLIB.Message {
  eeg_data: number[]
  emg_data: number[]
  time: number
  metadata: SampleMetadata
}

export interface EegBatch extends ROSLIB.Message {
  batch: Sample[]
}

export interface EegBatchMessage extends ROSLIB.Message {
  batch: Sample[]
}

export interface EegTrigger {
  index: number
  time: number
}

export interface EegTriggerMessage extends ROSLIB.Message {
  index: number
  time: number
}

export interface MTMSEvent {
  whenToExecute: number
  processingStartTime: number
  eventType: number
}

export interface MTMSEventMessage extends ROSLIB.Message {
  when_to_execute: number
  processing_start_time: number
  event_type: number
}
