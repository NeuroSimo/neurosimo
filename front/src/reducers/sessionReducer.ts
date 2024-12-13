import { createSlice, PayloadAction } from '@reduxjs/toolkit'
import { ChannelInfoWithEnabled } from 'types/pulseSequence'

const initialChannels: ChannelInfoWithEnabled[] = [
  {
    channelIndex: 1,
    enabled: true,
    voltage: 0,
  },
  {
    channelIndex: 2,
    enabled: false,
    voltage: 0,
  },
  {
    channelIndex: 3,
    enabled: false,
    voltage: 0,
  },
  {
    channelIndex: 4,
    enabled: false,
    voltage: 0,
  },
  {
    channelIndex: 5,
    enabled: false,
    voltage: 0,
  },
  {
    channelIndex: 6,
    enabled: false,
    voltage: 0,
  },
]

interface SessionState {
  description: string
  channels: ChannelInfoWithEnabled[]
  isis: number[]
  nofPulsesInBursts: number
  nofBurstsInTrains: number
  nofTrains: number
  iti: number
  ibi: number
}

const initialState: SessionState = {
  description: '',
  channels: initialChannels,
  isis: [],
  nofPulsesInBursts: 0,
  nofBurstsInTrains: 0,
  nofTrains: 0,
  iti: 0,
  ibi: 0,
}

const sessionSlice = createSlice({
  name: 'session',
  initialState,
  reducers: {
    setDescription: (state, action: PayloadAction<string>) => {
      state.description = action.payload
    },
    setChannels: (state, action: PayloadAction<ChannelInfoWithEnabled[]>) => {
      state.channels = action.payload
    },
    setIsis: (state, action: PayloadAction<number[]>) => {
      state.isis = action.payload
    },
    setNofPulsesInBursts: (state, action: PayloadAction<number>) => {
      state.nofPulsesInBursts = action.payload
    },
    setNofBurstsInTrains: (state, action: PayloadAction<number>) => {
      state.nofBurstsInTrains = action.payload
    },
    setNofTrains: (state, action: PayloadAction<number>) => {
      state.nofTrains = action.payload
    },
    setIti: (state, action: PayloadAction<number>) => {
      state.iti = action.payload
    },
    setIbi: (state, action: PayloadAction<number>) => {
      state.ibi = action.payload
    },
  },
})

export const {
  setDescription,
  setIbi,
  setIsis,
  setIti,
  setChannels,
  setNofPulsesInBursts,
  setNofBurstsInTrains,
  setNofTrains,
} = sessionSlice.actions

export default sessionSlice.reducer
