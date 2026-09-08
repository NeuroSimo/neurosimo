import React, { ReactNode } from 'react'

import { useSystemConfig } from './SystemConfigProvider'
import { useProjectFilenameList } from 'utils/useProjectFilenameList'

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
  const { activeProject } = useSystemConfig()

  const recordingsList = useProjectFilenameList('/neurosimo/recording/recordings/list', activeProject)

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