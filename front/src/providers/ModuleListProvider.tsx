import React, { useState, useEffect, ReactNode } from 'react'
import { Topic } from '@foxglove/roslibjs'

import { ros } from 'ros/ros'
import { useSessionConfig } from './SessionConfigProvider'

export interface FilenameList extends ROSLIB.Message {
  filenames: string[]
}

interface ModuleListContextType {
  preprocessorList: string[]
  preprocessorModule: string
  preprocessorEnabled: boolean

  deciderList: string[]
  deciderModule: string
  deciderEnabled: boolean

  presenterList: string[]
  presenterModule: string
  presenterEnabled: boolean

  protocolList: string[]
  protocolName: string
}

const defaultModuleListState: ModuleListContextType = {
  preprocessorList: [],
  preprocessorModule: '',
  preprocessorEnabled: false,

  deciderList: [],
  deciderModule: '',
  deciderEnabled: false,

  presenterList: [],
  presenterModule: '',
  presenterEnabled: false,

  protocolList: [],
  protocolName: '',
}

export const ModuleListContext = React.createContext<ModuleListContextType>(defaultModuleListState)

interface ModuleListProviderProps {
  children: ReactNode
}

export const ModuleListProvider: React.FC<ModuleListProviderProps> = ({ children }) => {
  const { pipeline } = useSessionConfig()

  const [preprocessorList, setPreprocessorList] = useState<string[]>([])
  const [deciderList, setDeciderList] = useState<string[]>([])
  const [presenterList, setPresenterList] = useState<string[]>([])
  const [protocolList, setProtocolList] = useState<string[]>([])

  // Get parameter values from structured parameter store
  const preprocessorModule = pipeline.preprocessor.module
  const preprocessorEnabled = pipeline.preprocessor.enabled
  const deciderModule = pipeline.decider.module
  const deciderEnabled = pipeline.decider.enabled
  const presenterModule = pipeline.presenter.module
  const presenterEnabled = pipeline.presenter.enabled
  const protocolName = pipeline.experiment.protocol

  useEffect(() => {
    /* Subscriber for preprocessor list. */
    const preprocessorListSubscriber = new Topic<FilenameList>({
      ros: ros,
      name: '/pipeline/preprocessor/list',
      messageType: 'project_interfaces/FilenameList',
    })

    preprocessorListSubscriber.subscribe((message) => {
      setPreprocessorList(message.filenames)
    })

    /* Subscriber for decider list. */
    const deciderListSubscriber = new Topic<FilenameList>({
      ros: ros,
      name: '/pipeline/decider/list',
      messageType: 'project_interfaces/FilenameList',
    })

    deciderListSubscriber.subscribe((message) => {
      setDeciderList(message.filenames)
    })

    /* Subscriber for presenter list. */
    const presenterListSubscriber = new Topic<FilenameList>({
      ros: ros,
      name: '/pipeline/presenter/list',
      messageType: 'project_interfaces/FilenameList',
    })

    presenterListSubscriber.subscribe((message) => {
      setPresenterList(message.filenames)
    })

    /* Subscriber for available protocols. */
    const protocolListSubscriber = new Topic<FilenameList>({
      ros: ros,
      name: '/experiment/protocol/list',
      messageType: 'project_interfaces/FilenameList',
    })

    protocolListSubscriber.subscribe((message) => {
      setProtocolList(message.filenames)
    })

    /* Unsubscribers */
    return () => {
      preprocessorListSubscriber.unsubscribe()
      deciderListSubscriber.unsubscribe()
      presenterListSubscriber.unsubscribe()
      protocolListSubscriber.unsubscribe()
    }
  }, [])

  return (
    <ModuleListContext.Provider
      value={{
        preprocessorList,
        preprocessorModule,
        preprocessorEnabled,
        deciderList,
        deciderModule,
        deciderEnabled,
        presenterList,
        presenterModule,
        presenterEnabled,
        protocolList,
        protocolName,
      }}
    >
      {children}
    </ModuleListContext.Provider>
  )
}
