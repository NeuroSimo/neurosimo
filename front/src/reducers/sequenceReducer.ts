import { createSlice, PayloadAction } from '@reduxjs/toolkit'
import { PulseSequence } from 'types/pulseSequence'

interface SequenceState {
  sequences: PulseSequence[]
  expandedSequences: Record<string, boolean>
}

interface SetExpandedSequencePayload {
  index: number
  expanded: boolean
}

const initialState: SequenceState = {
  sequences: [],
  expandedSequences: {},
}

const sequenceSlice = createSlice({
  name: 'sequences',
  initialState,
  reducers: {
    setPulseSequences: (state, action: PayloadAction<PulseSequence[]>) => {
      state.sequences = action.payload
    },
    setExpandedSequences: (state, action: PayloadAction<Record<string, boolean>>) => {
      state.expandedSequences = action.payload
    },
    setExpandedSequence: (state, action: PayloadAction<SetExpandedSequencePayload>) => {
      state.expandedSequences[action.payload.index] = action.payload.expanded
    },
  },
})

export const { setExpandedSequence, setPulseSequences } = sequenceSlice.actions
export default sequenceSlice.reducer
