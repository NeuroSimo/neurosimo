import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from 'roslib'

import { ros } from 'ros/ros'

interface EegInfo extends Message {
  sampling_frequency: number
  num_of_eeg_channels: number
  num_of_emg_channels: number
}

interface Int32 extends Message {
  data: number
}

interface EegStatistics extends Message {
  num_of_raw_samples: number
  max_time_between_raw_samples: number

  num_of_preprocessed_samples: number
  max_time_between_preprocessed_samples: number

  preprocessing_time_max: number
  preprocessing_time_q95: number
  preprocessing_time_median: number
}

interface EegContextType {
  eegInfo: EegInfo | null
  eegStatistics: EegStatistics | null
  droppedSamples: number | null
}

const defaultEegState: EegContextType = {
  eegInfo: null,
  eegStatistics: null,
  droppedSamples: null,
}

export const EegContext = React.createContext<EegContextType>(defaultEegState)

interface EegProviderProps {
  children: ReactNode
}

export const EegProvider: React.FC<EegProviderProps> = ({ children }) => {
  const [eegInfo, setEegInfo] = useState<EegInfo | null>(null)
  const [eegStatistics, setEegStatistics] = useState<EegStatistics | null>(null)
  const [droppedSamples, setDroppedSamples] = useState<number | null>(null)

  useEffect(() => {
    /* Subscriber for EEG info. */
    const eegInfoSubscriber = new Topic<EegInfo>({
      ros: ros,
      name: '/eeg/info',
      messageType: 'eeg_interfaces/EegInfo',
    })

    eegInfoSubscriber.subscribe((message) => {
      setEegInfo(message)
    })

    /* Subscriber for EEG statistics. */
    const eegStatisticsSubscriber = new Topic<EegStatistics>({
      ros: ros,
      name: '/eeg/statistics',
      messageType: 'eeg_interfaces/EegStatistics',
    })

    eegStatisticsSubscriber.subscribe((message) => {
      setEegStatistics(message)
    })

    /* Subscriber for dropped sample count. */
    const droppedSamplesSubscriber = new Topic<Int32>({
      ros: ros,
      name: '/pipeline/dropped_samples',
      messageType: 'std_msgs/Int32',
    })

    droppedSamplesSubscriber.subscribe((message) => {
      console.log('Dropped samples:', message.data)
      setDroppedSamples(message.data)
    })

    /* Unsubscribers */
    return () => {
      eegInfoSubscriber.unsubscribe()
      eegStatisticsSubscriber.unsubscribe()
      droppedSamplesSubscriber.unsubscribe()
    }
  }, [])

  return <EegContext.Provider value={{ eegInfo, eegStatistics, droppedSamples }}>{children}</EegContext.Provider>
}
