import React, { useState, useEffect, ReactNode } from 'react'
import { Topic } from '@foxglove/roslibjs'

import { ros } from 'ros/ros'
import { FilenameList } from './PipelineConfigProvider'

interface PlaybackContextType {
  recordingsList: string[]
}

const defaultPlaybackState: PlaybackContextType = {
  recordingsList: [],
}

export const PlaybackContext = React.createContext<PlaybackContextType>(defaultPlaybackState)

interface PlaybackProviderProps {
  children: ReactNode
}

export const PlaybackProvider: React.FC<PlaybackProviderProps> = ({ children }) => {
  const [recordingsList, setRecordingsList] = useState<string[]>([])

  useEffect(() => {
    /* Subscriber for recordings list. */
    const recordingsListSubscriber = new Topic<FilenameList>({
      ros: ros,
      name: '/playback/recordings/list',
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
    <PlaybackContext.Provider
      value={{
        recordingsList,
      }}
    >
      {children}
    </PlaybackContext.Provider>
  )
}