import { createSlice, PayloadAction } from '@reduxjs/toolkit'
import { Target } from 'types/target'

interface TargetState {
  targets: Target[]
}

const initialState: TargetState = {
  targets: [],
}

const targetSlice = createSlice({
  name: 'targets',
  initialState,
  reducers: {
    setTargets: (state, action: PayloadAction<Target[]>) => {
      state.targets = action.payload
    },
  },
})

export const { setTargets } = targetSlice.actions
export default targetSlice.reducer
