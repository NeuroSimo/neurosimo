import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from 'roslib'

import { ros } from 'ros/ros'

interface Dataset extends ROSLIB.Message {
  name: string
  json_filename: string
  data_filename: string
  sampling_frequency: number
  num_eeg_channels: number
  num_emg_channels: number
  duration: number
  loop: boolean
  pulse_count: number
}

interface DatasetList extends ROSLIB.Message {
  datasets: Dataset[]
}

interface RosString extends ROSLIB.Message {
  data: string
}

interface RosBoolean extends ROSLIB.Message {
  data: boolean
}

interface RosFloat64 extends ROSLIB.Message {
  data: number
}

interface EegSimulatorContextType {
  datasetList: Dataset[]
  dataset: string
  enabled: boolean
  recordData: boolean
  startTime: number
}

const defaultDatasetState: EegSimulatorContextType = {
  datasetList: [],
  dataset: '',
  enabled: false,
  recordData: false,
  startTime: 0,
}

export const EegSimulatorContext = React.createContext<EegSimulatorContextType>(defaultDatasetState)

interface EegSimulatorProviderProps {
  children: ReactNode
}

export const EegSimulatorProvider: React.FC<EegSimulatorProviderProps> = ({ children }) => {
  const [datasetList, setDatasetList] = useState<Dataset[]>([])
  const [dataset, setDataset] = useState<string>('')

  const [enabled, setEnabled] = useState<boolean>(false)
  const [startTime, setStartTime] = useState<number>(0)
  const [recordData, setRecordData] = useState<boolean>(false)

  useEffect(() => {
    /* Subscriber for dataset list. */
    const datasetListSubscriber = new Topic<DatasetList>({
      ros: ros,
      name: '/eeg_simulator/dataset/list',
      messageType: 'project_interfaces/DatasetList',
    })

    datasetListSubscriber.subscribe((message) => {
      setDatasetList(message.datasets)
    })

    /* Subscriber for active dataset. */
    const datasetSubscriber = new Topic<RosString>({
      ros: ros,
      name: '/eeg_simulator/dataset',
      messageType: 'std_msgs/String',
    })

    datasetSubscriber.subscribe((message) => {
      setDataset(message.data)
    })

    /* Subscriber for enabled. */
    const enabledSubscriber = new Topic<RosBoolean>({
      ros: ros,
      name: '/eeg_simulator/enabled',
      messageType: 'std_msgs/Bool',
    })

    enabledSubscriber.subscribe((message) => {
      setEnabled(message.data)
    })

    /* Subscriber for start time. */
    const startTimeSubscriber = new Topic<RosFloat64>({
      ros: ros,
      name: '/eeg_simulator/start_time',
      messageType: 'std_msgs/Float64',
    })

    startTimeSubscriber.subscribe((message) => {
      setStartTime(message.data)
    })

    /* Subscriber for record data. */
    const recordDataSubscriber = new Topic<RosBoolean>({
      ros: ros,
      name: '/eeg_recorder/record_data',
      messageType: 'std_msgs/Bool',
    })

    recordDataSubscriber.subscribe((message) => {
      setRecordData(message.data)
    })

    /* Unsubscribers */
    return () => {
      datasetListSubscriber.unsubscribe()
      datasetSubscriber.unsubscribe()

      enabledSubscriber.unsubscribe()
      startTimeSubscriber.unsubscribe()
      recordDataSubscriber.unsubscribe()
    }
  }, [])

  return (
    <EegSimulatorContext.Provider
      value={{
        datasetList,
        dataset,
        enabled,
        startTime,
        recordData,
      }}
    >
      {children}
    </EegSimulatorContext.Provider>
  )
}
