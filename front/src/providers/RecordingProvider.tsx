import React, { useState, useEffect, ReactNode } from 'react'
import { Topic } from '@foxglove/roslibjs'

import { ros } from 'ros/ros'
import { FilenameList } from './ModuleListProvider'

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
  const [recordingsList, setRecordingsList] = useState<string[]>([])

  useEffect(() => {
    /* Subscriber for recordings list. */
    const recordingsListSubscriber = new Topic<FilenameList>({
      ros: ros,
      name: '/recording/recordings/list',
      messageType: 'project_interfaces/FilenameList',
    })

    recordingsListSubscriber.subscribe((message: FilenameList) => {
      setRecordingsList(message.filenames)
    })

    /* Unsubscriber */
    return () => {
      recordingsListSubscriber.unsubscribe()
    }
  }, [])

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