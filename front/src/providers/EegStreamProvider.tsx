import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from 'roslib'
import { ros } from 'ros/ros'

interface EegDeviceInfo extends Message {
  is_streaming: boolean
  sampling_frequency: number
  num_eeg_channels: number
  num_emg_channels: number
}

interface EegStreamContextType {
  eegDeviceInfo: EegDeviceInfo | null
}

const defaultEegStreamState: EegStreamContextType = {
  eegDeviceInfo: null,
}

export const EegStreamContext = React.createContext<EegStreamContextType>(defaultEegStreamState)

interface EegStreamProviderProps {
  children: ReactNode
}

export const EegStreamProvider: React.FC<EegStreamProviderProps> = ({ children }) => {
  const [eegDeviceInfo, setEegDeviceInfo] = useState<EegDeviceInfo | null>(null)

  useEffect(() => {
    const eegInfoSubscriber = new Topic<EegDeviceInfo>({
      ros: ros,
      name: '/eeg_device/info',
      messageType: 'eeg_interfaces/EegDeviceInfo',
    })

    eegInfoSubscriber.subscribe((message) => {
      setEegDeviceInfo(message)
    })

    return () => {
      eegInfoSubscriber.unsubscribe()
    }
  }, [])

  return <EegStreamContext.Provider value={{ eegDeviceInfo }}>{children}</EegStreamContext.Provider>
}
