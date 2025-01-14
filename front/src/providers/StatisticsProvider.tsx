import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from 'roslib'
import { ros } from 'ros/ros'

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

interface StatisticsContextType {
  eegStatistics: EegStatistics | null
  droppedSamples: number | null
}

const defaultStatisticsState: StatisticsContextType = {
  eegStatistics: null,
  droppedSamples: null,
}

export const StatisticsContext = React.createContext<StatisticsContextType>(defaultStatisticsState)

interface StatisticsProviderProps {
  children: ReactNode
}

export const StatisticsProvider: React.FC<StatisticsProviderProps> = ({ children }) => {
  const [eegStatistics, setEegStatistics] = useState<EegStatistics | null>(null)
  const [droppedSamples, setDroppedSamples] = useState<number | null>(null)

  useEffect(() => {
    const eegStatisticsSubscriber = new Topic<EegStatistics>({
      ros: ros,
      name: '/eeg/statistics',
      messageType: 'eeg_interfaces/EegStatistics',
    })

    const droppedSamplesSubscriber = new Topic<Int32>({
      ros: ros,
      name: '/pipeline/dropped_samples',
      messageType: 'std_msgs/Int32',
    })

    eegStatisticsSubscriber.subscribe((message) => {
      setEegStatistics(message)
    })

    droppedSamplesSubscriber.subscribe((message) => {
      console.log('Dropped samples:', message.data)
      setDroppedSamples(message.data)
    })

    return () => {
      eegStatisticsSubscriber.unsubscribe()
      droppedSamplesSubscriber.unsubscribe()
    }
  }, [])

  return <StatisticsContext.Provider value={{ eegStatistics, droppedSamples }}>{children}</StatisticsContext.Provider>
}
