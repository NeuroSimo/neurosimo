import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from 'roslib'

import { ros } from 'ros/ros'
import { startSimulatorRos, stopSimulatorRos } from 'ros/eeg_simulator'

export enum StreamerStateValue {
  READY = 0,
  LOADING = 1,
  RUNNING = 2,
  ERROR = 3,
}

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

interface RosStreamerState extends ROSLIB.Message {
  state: StreamerStateValue
}

interface EegSimulatorContextType {
  datasetList: Dataset[]
  dataset: string
  recordData: boolean
  startTime: number
  streamerState: StreamerStateValue
  toggleStreaming: () => void
}

const defaultDatasetState: EegSimulatorContextType = {
  datasetList: [],
  dataset: '',
  recordData: false,
  startTime: 0,
  streamerState: StreamerStateValue.READY,
  toggleStreaming: () => {},
}

export const EegSimulatorContext = React.createContext<EegSimulatorContextType>(defaultDatasetState)

interface EegSimulatorProviderProps {
  children: ReactNode
}

export const EegSimulatorProvider: React.FC<EegSimulatorProviderProps> = ({ children }) => {
  const [datasetList, setDatasetList] = useState<Dataset[]>([])
  const [dataset, setDataset] = useState<string>('')

  const [startTime, setStartTime] = useState<number>(0)
  const [recordData, setRecordData] = useState<boolean>(false)
  const [streamerState, setStreamerState] = useState<StreamerStateValue>(StreamerStateValue.READY)

  useEffect(() => {
    /* Subscriber for dataset list. */
    const datasetListSubscriber = new Topic<DatasetList>({
      ros: ros,
      name: '/eeg_simulator/dataset/list',
      messageType: 'project_interfaces/DatasetList',
    })

    datasetListSubscriber.subscribe((message: DatasetList) => {
      setDatasetList(message.datasets)
    })

    /* Subscriber for active dataset. */
    const datasetSubscriber = new Topic<RosString>({
      ros: ros,
      name: '/eeg_simulator/dataset',
      messageType: 'std_msgs/String',
    })

    datasetSubscriber.subscribe((message: RosString) => {
      setDataset(message.data)
    })

    /* Subscriber for start time. */
    const startTimeSubscriber = new Topic<RosFloat64>({
      ros: ros,
      name: '/eeg_simulator/start_time',
      messageType: 'std_msgs/Float64',
    })

    startTimeSubscriber.subscribe((message: RosFloat64) => {
      setStartTime(message.data)
    })

    /* Subscriber for record data. */
    const recordDataSubscriber = new Topic<RosBoolean>({
      ros: ros,
      name: '/eeg_recorder/record_data',
      messageType: 'std_msgs/Bool',
    })

    recordDataSubscriber.subscribe((message: RosBoolean) => {
      setRecordData(message.data)
    })

    /* Subscriber for simulator state. */
    const stateSubscriber = new Topic<RosStreamerState>({
      ros: ros,
      name: '/eeg_simulator/state',
      messageType: 'system_interfaces/StreamerState',
    })

    stateSubscriber.subscribe((message: RosStreamerState) => {
      setStreamerState(message.state)
    })

    /* Unsubscribers */
    return () => {
      datasetListSubscriber.unsubscribe()
      datasetSubscriber.unsubscribe()

      startTimeSubscriber.unsubscribe()
      recordDataSubscriber.unsubscribe()
      stateSubscriber.unsubscribe()
    }
  }, [])

  const toggleStreaming = () => {
    if (streamerState === StreamerStateValue.RUNNING) {
      // Service call to stop
      stopSimulatorRos(() => {})
    } else {
      // Service call to start
      startSimulatorRos(() => {})
    }
  }

  return (
    <EegSimulatorContext.Provider
      value={{
        datasetList,
        dataset,
        startTime,
        recordData,
        streamerState,
        toggleStreaming,
      }}
    >
      {children}
    </EegSimulatorContext.Provider>
  )
}
