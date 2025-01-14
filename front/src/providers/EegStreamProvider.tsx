import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from 'roslib'
import { ros } from 'ros/ros'

interface EegInfo extends Message {
  sampling_frequency: number
  num_of_eeg_channels: number
  num_of_emg_channels: number
}

interface EegStreamContextType {
  eegInfo: EegInfo | null
}

const defaultEegStreamState: EegStreamContextType = {
  eegInfo: null,
}

export const EegStreamContext = React.createContext<EegStreamContextType>(defaultEegStreamState)

interface EegStreamProviderProps {
  children: ReactNode
}

export const EegStreamProvider: React.FC<EegStreamProviderProps> = ({ children }) => {
  const [eegInfo, setEegInfo] = useState<EegInfo | null>(null)

  useEffect(() => {
    const eegInfoSubscriber = new Topic<EegInfo>({
      ros: ros,
      name: '/eeg/info',
      messageType: 'eeg_msgs/EegInfo',
    })

    eegInfoSubscriber.subscribe((message) => {
      setEegInfo(message)
    })

    return () => {
      eegInfoSubscriber.unsubscribe()
    }
  }, [])

  return <EegStreamContext.Provider value={{ eegInfo }}>{children}</EegStreamContext.Provider>
}
