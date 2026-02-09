import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from '@foxglove/roslibjs'

import { ros } from 'ros/ros'
import { useSessionConfig } from './SessionConfigProvider'
import { FilenameList } from './ModuleListProvider'

export enum DataSourceStateValue {
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
  datasets: string[]
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

interface RosDataSourceState extends ROSLIB.Message {
  state: DataSourceStateValue
}

interface EegSimulatorContextType {
  datasetList: string[]
  dataset: string
  startTime: number
  dataSourceState: DataSourceStateValue
}

const defaultDatasetState: EegSimulatorContextType = {
  datasetList: [],
  dataset: '',
  startTime: 0,
  dataSourceState: DataSourceStateValue.READY,
}

export const EegSimulatorContext = React.createContext<EegSimulatorContextType>(defaultDatasetState)

interface EegSimulatorProviderProps {
  children: ReactNode
}

export const EegSimulatorProvider: React.FC<EegSimulatorProviderProps> = ({ children }) => {
  const { simulator } = useSessionConfig()

  const [datasetList, setDatasetList] = useState<string[]>([])
  const [dataSourceState, setDataSourceState] = useState<DataSourceStateValue>(DataSourceStateValue.READY)

  // Get parameter values from structured parameter store
  const dataset = simulator.dataset_filename
  const startTime = simulator.start_time

  useEffect(() => {
    /* Subscriber for dataset list. */
    const datasetListSubscriber = new Topic<FilenameList>({
      ros: ros,
      name: '/eeg_simulator/dataset/list',
      messageType: 'project_interfaces/FilenameList',
    })

    datasetListSubscriber.subscribe((message: FilenameList) => {
      setDatasetList(message.filenames)
    })

    /* Subscriber for simulator state. */
    const stateSubscriber = new Topic<RosDataSourceState>({
      ros: ros,
      name: '/eeg_simulator/state',
      messageType: 'system_interfaces/DataSourceState',
    })

    stateSubscriber.subscribe((message: RosDataSourceState) => {
      setDataSourceState(message.state)
    })

    /* Unsubscribers */
    return () => {
      datasetListSubscriber.unsubscribe()
      stateSubscriber.unsubscribe()
    }
  }, [])

  return (
    <EegSimulatorContext.Provider
      value={{
        datasetList,
        dataset,
        startTime,
        dataSourceState,
      }}
    >
      {children}
    </EegSimulatorContext.Provider>
  )
}
