import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from 'roslib'

import { ros } from 'ros/ros'

interface Dataset extends ROSLIB.Message {
  name: string
  json_filename: string
  data_filename: string
  sampling_frequency: number
  num_of_eeg_channels: number
  num_of_emg_channels: number
  duration: number
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

interface EegSimulatorContextType {
  datasetList: Dataset[]
  dataset: string
  playback: boolean
  loop: boolean
  recordData: boolean
}

const defaultDatasetState: EegSimulatorContextType = {
  datasetList: [],
  dataset: '',
  playback: false,
  loop: false,
  recordData: false,
}

export const EegSimulatorContext = React.createContext<EegSimulatorContextType>(defaultDatasetState)

interface EegSimulatorProviderProps {
  children: ReactNode
}

export const EegSimulatorProvider: React.FC<EegSimulatorProviderProps> = ({ children }) => {
  const [datasetList, setDatasetList] = useState<Dataset[]>([])
  const [dataset, setDataset] = useState<string>('')

  const [playback, setPlayback] = useState<boolean>(false)
  const [loop, setLoop] = useState<boolean>(false)
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

    /* Subscriber for playback. */
    const playbackSubscriber = new Topic<RosBoolean>({
      ros: ros,
      name: '/eeg_simulator/playback',
      messageType: 'std_msgs/Bool',
    })

    playbackSubscriber.subscribe((message) => {
      setPlayback(message.data)
    })

    /* Subscriber for loop. */
    const loopSubscriber = new Topic<RosBoolean>({
      ros: ros,
      name: '/eeg_simulator/loop',
      messageType: 'std_msgs/Bool',
    })

    loopSubscriber.subscribe((message) => {
      setLoop(message.data)
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

      playbackSubscriber.unsubscribe()
      loopSubscriber.unsubscribe()
      recordDataSubscriber.unsubscribe()
    }
  }, [])

  return (
    <EegSimulatorContext.Provider
      value={{
        datasetList,
        dataset,
        playback,
        loop,
        recordData,
      }}
    >
      {children}
    </EegSimulatorContext.Provider>
  )
}
