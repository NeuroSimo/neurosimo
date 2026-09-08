import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from '@foxglove/roslibjs'

import { ros } from 'ros/ros'
import { useSessionConfig } from './SessionConfigProvider'
import { useSystemConfig } from './SystemConfigProvider'
import { useProjectFilenameList } from 'utils/useProjectFilenameList'

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
  trial_count: number
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
  externalRecordingsList: string[]
  dataset: string
  startTime: number
  playbackSpeed: number
  dataSourceState: DataSourceStateValue
}

const defaultDatasetState: EegSimulatorContextType = {
  datasetList: [],
  externalRecordingsList: [],
  dataset: '',
  startTime: 0,
  playbackSpeed: 1,
  dataSourceState: DataSourceStateValue.READY,
}

export const EegSimulatorContext = React.createContext<EegSimulatorContextType>(defaultDatasetState)

interface EegSimulatorProviderProps {
  children: ReactNode
}

export const EegSimulatorProvider: React.FC<EegSimulatorProviderProps> = ({ children }) => {
  const { simulator } = useSessionConfig()
  const { activeProject } = useSystemConfig()

  const datasetList = useProjectFilenameList('/neurosimo/eeg_simulator/dataset/list', activeProject)
  const externalRecordingsList = useProjectFilenameList(
    '/neurosimo/eeg_simulator/external_recordings/list',
    activeProject,
  )
  const [dataSourceState, setDataSourceState] = useState<DataSourceStateValue>(DataSourceStateValue.READY)

  const dataset = simulator.dataset_filename
  const startTime = simulator.start_time
  const playbackSpeed = simulator.playback_speed

  useEffect(() => {
    /* Subscriber for simulator state. */
    const stateSubscriber = new Topic<RosDataSourceState>({
      ros: ros,
      name: '/neurosimo/eeg_simulator/state',
      messageType: 'neurosimo_system_interfaces/DataSourceState',
    })

    stateSubscriber.subscribe((message: RosDataSourceState) => {
      setDataSourceState(message.state)
    })

    /* Unsubscriber */
    return () => {
      stateSubscriber.unsubscribe()
    }
  }, [])

  return (
    <EegSimulatorContext.Provider
      value={{
        datasetList,
        externalRecordingsList,
        dataset,
        startTime,
        playbackSpeed,
        dataSourceState,
      }}
    >
      {children}
    </EegSimulatorContext.Provider>
  )
}
