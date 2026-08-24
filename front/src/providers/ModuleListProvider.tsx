import React, { useState, useEffect, useRef, ReactNode } from 'react'
import { Topic } from '@foxglove/roslibjs'

import { ros } from 'ros/ros'
import { useSessionConfig, RuntimeParameterValue } from './SessionConfigProvider'
import { useGlobalConfig } from './GlobalConfigProvider'
import { getProtocolInfoRos, RuntimeParameterInfo } from 'ros/experiment'

export interface FilenameList extends ROSLIB.Message {
  filenames: string[]
}

/* A runtime parameter counts as "set" when the user has provided a usable value.
   Booleans always have a value (the checkbox is either on or off). */
const isRuntimeParameterSet = (
  descriptor: RuntimeParameterInfo,
  value: RuntimeParameterValue | undefined,
): boolean => {
  if (descriptor.type === 'bool') {
    return true
  }
  if (value === undefined || value === null) {
    return false
  }
  if (descriptor.type === 'string') {
    return String(value).trim() !== ''
  }
  /* Numeric types (float, int). */
  return typeof value === 'number' && !Number.isNaN(value)
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

  /* Runtime parameter descriptors for the selected protocol, plus whether every
     one of them currently has a usable value (all are required). */
  runtimeParameterInfos: RuntimeParameterInfo[]
  runtimeParametersValid: boolean
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

  runtimeParameterInfos: [],
  runtimeParametersValid: true,
}

export const ModuleListContext = React.createContext<ModuleListContextType>(defaultModuleListState)

interface ModuleListProviderProps {
  children: ReactNode
}

export const ModuleListProvider: React.FC<ModuleListProviderProps> = ({ children }) => {
  const { pipeline, runtimeParameters, setRuntimeParameters } = useSessionConfig()
  const { activeProject } = useGlobalConfig()

  /* Keep the latest runtime-parameter values in a ref so the fetch effect can prune
     stale entries without having to depend on (and thus refetch on) every value edit. */
  const runtimeParametersRef = useRef(runtimeParameters)
  useEffect(() => {
    runtimeParametersRef.current = runtimeParameters
  }, [runtimeParameters])

  const [preprocessorList, setPreprocessorList] = useState<string[]>([])
  const [deciderList, setDeciderList] = useState<string[]>([])
  const [presenterList, setPresenterList] = useState<string[]>([])
  const [protocolList, setProtocolList] = useState<string[]>([])
  const [runtimeParameterInfos, setRuntimeParameterInfos] = useState<RuntimeParameterInfo[]>([])

  // Get parameter values from structured parameter store
  const preprocessorModule = pipeline.preprocessor.module
  const preprocessorEnabled = pipeline.preprocessor.enabled
  const deciderModule = pipeline.decider.module
  const deciderEnabled = pipeline.decider.enabled
  const presenterModule = pipeline.presenter.module
  const presenterEnabled = pipeline.presenter.enabled
  const protocolName = pipeline.experiment.protocol

  /* Fetch the runtime parameter descriptors whenever the selected protocol changes.
     protocolList is also a dependency: the backend re-publishes the protocol list
     (giving a new array reference) on any change to the protocols directory, including
     in-place edits of the currently selected protocol file. Re-fetching on that keeps
     the runtime-parameter UI in sync without having to switch protocols to refresh. */
  useEffect(() => {
    if (!protocolName || protocolName.trim() === '' || !activeProject) {
      setRuntimeParameterInfos([])
      return
    }

    getProtocolInfoRos(activeProject, protocolName, (info) => {
      const infos = info?.runtime_parameters ?? []
      setRuntimeParameterInfos(infos)

      /* Drop any stored values whose descriptor no longer exists in the protocol
         (e.g. a parameter was removed or renamed on disk) so the session config does
         not keep stale entries around. */
      const validNames = new Set(infos.map((descriptor) => descriptor.name))
      const currentValues = runtimeParametersRef.current
      const staleNames = Object.keys(currentValues).filter((name) => !validNames.has(name))
      if (staleNames.length > 0) {
        const pruned = { ...currentValues }
        staleNames.forEach((name) => delete pruned[name])
        setRuntimeParameters(pruned)
      }
    })
  }, [protocolName, activeProject, protocolList])

  /* Every runtime parameter is required, so the session can only start once all of
     them have a usable value. */
  const runtimeParametersValid = runtimeParameterInfos.every((descriptor) =>
    isRuntimeParameterSet(descriptor, runtimeParameters[descriptor.name]),
  )

  useEffect(() => {
    /* Subscriber for preprocessor list. */
    const preprocessorListSubscriber = new Topic<FilenameList>({
      ros: ros,
      name: '/neurosimo/pipeline/preprocessor/list',
      messageType: 'neurosimo_project_interfaces/FilenameList',
    })

    preprocessorListSubscriber.subscribe((message) => {
      setPreprocessorList(message.filenames)
    })

    /* Subscriber for decider list. */
    const deciderListSubscriber = new Topic<FilenameList>({
      ros: ros,
      name: '/neurosimo/pipeline/decider/list',
      messageType: 'neurosimo_project_interfaces/FilenameList',
    })

    deciderListSubscriber.subscribe((message) => {
      setDeciderList(message.filenames)
    })

    /* Subscriber for presenter list. */
    const presenterListSubscriber = new Topic<FilenameList>({
      ros: ros,
      name: '/neurosimo/pipeline/presenter/list',
      messageType: 'neurosimo_project_interfaces/FilenameList',
    })

    presenterListSubscriber.subscribe((message) => {
      setPresenterList(message.filenames)
    })

    /* Subscriber for available protocols. */
    const protocolListSubscriber = new Topic<FilenameList>({
      ros: ros,
      name: '/neurosimo/experiment/protocol/list',
      messageType: 'neurosimo_project_interfaces/FilenameList',
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
        runtimeParameterInfos,
        runtimeParametersValid,
      }}
    >
      {children}
    </ModuleListContext.Provider>
  )
}
