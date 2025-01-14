export interface SessionMessage {
  session: ISession
}

export interface PulseSequence {
  name: string
  comment: string
  visible: boolean
  selected: boolean
  nofPulses: number
  channelInfo: ChannelInfo[]
  pulses: Pulse[]
  intensity: number
  isi: number // only here because it's in the table, used to set isi for all pulses
}

export interface Train {
  sequences: PulseSequence[]
  nofBursts: number
  ibi: number
}

export interface ISession {
  train: Train
  nofTrains: number
  iti: number
  description: string
}

export interface Pulse {
  targetIndex: number
  intensity: number
  isi: number
  modeDuration: number
  visible: boolean
  selected: boolean
}

interface ChannelInfo {
  channelIndex: number
  voltage: number
}

export interface ChannelInfoWithEnabled {
  channelIndex: number
  voltage: number
  enabled: boolean
}

export const pulseSequenceChangeableKeys = ['name', 'selected', 'isi', 'intensity'] as const
export type PulseSequenceChangeableKey = (typeof pulseSequenceChangeableKeys)[number]

export const pulseChangeableKeys = ['selected', 'isi', 'intensity', 'visible'] as const
export type PulseChangeableKey = (typeof pulseChangeableKeys)[number]
