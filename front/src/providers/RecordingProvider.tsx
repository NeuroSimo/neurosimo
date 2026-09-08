import React, { ReactNode } from 'react'

import { useProjectFiles } from './ProjectFilesProvider'

interface RecordingContextType {
  recordingsList: string[]
}

const defaultRecordingState: RecordingContextType = {
  recordingsList: [],
}

export const RecordingContext = React.createContext<RecordingContextType>(defaultRecordingState)

interface RecordingProviderProps {
  children: ReactNode
}

export const RecordingProvider: React.FC<RecordingProviderProps> = ({ children }) => {
  const { recordingsList } = useProjectFiles()

  return (
    <RecordingContext.Provider
      value={{
        recordingsList,
      }}
    >
      {children}
    </RecordingContext.Provider>
  )
}