import React, { useState, useEffect, ReactNode } from 'react'
import { Topic } from '@foxglove/roslibjs'

import { ros } from 'ros/ros'
import { useSessionConfig, RuntimeParameterValue } from './SessionConfigProvider'
import { useGlobalConfig } from './GlobalConfigProvider'
import { getProtocolInfoRos, RuntimeParameterInfo } from 'ros/experiment'

export interface FilenameList extends ROSLIB.Message {
  filenames: string[]
}

/* A runtime parameter counts as "set" when the user has provided a usable value.
   Booleans are usable as soon as they have a value; they are defaulted to false rather
   than being asked from the user. */
const isRuntimeParameterSet = (
  descriptor: RuntimeParameterInfo,
  value: RuntimeParameterValue | undefined,
): boolean => {
  if (value === undefined || value === null) {
    return false
  }
  if (descriptor.type === 'bool') {
    return typeof value === 'boolean'
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

  const [preprocessorList, setPreprocessorList] = useState<string[]>([])
  const [deciderList, setDeciderList] = useState<string[]>([])
  const [presenterList, setPresenterList] = useState<string[]>([])
  const [protocolList, setProtocolList] = useState<string[]>([])

  /* Runtime parameter descriptors, tagged with the project/protocol they were fetched for.
     The tag is needed because the fetch is asynchronous: while switching projects the
     descriptors briefly still describe the previously selected protocol, and applying
     them to the newly loaded session config would write the wrong parameters.

     null means the descriptors for the current protocol are not known: either nothing has
     been fetched yet, or the fetch failed. That is deliberately distinct from a successful
     fetch returning an empty list, which means the protocol declares no runtime parameters. */
  const [fetchedRuntimeParameters, setFetchedRuntimeParameters] = useState<{
    key: string
    infos: RuntimeParameterInfo[]
  } | null>(null)

  // Get parameter values from structured parameter store
  const preprocessorModule = pipeline.preprocessor.module
  const preprocessorEnabled = pipeline.preprocessor.enabled
  const deciderModule = pipeline.decider.module
  const deciderEnabled = pipeline.decider.enabled
  const presenterModule = pipeline.presenter.module
  const presenterEnabled = pipeline.presenter.enabled
  const protocolName = pipeline.experiment.protocol

  /* Identifies the protocol whose descriptors are currently relevant. */
  const protocolKey = activeProject && protocolName.trim() !== '' ? `${activeProject}/${protocolName}` : ''

  /* Fetch the runtime parameter descriptors whenever the selected protocol changes.
     protocolList is also a dependency: the backend re-publishes the protocol list
     (giving a new array reference) on any change to the protocols directory, including
     in-place edits of the currently selected protocol file. Re-fetching on that keeps
     the runtime-parameter UI in sync without having to switch protocols to refresh. */
  useEffect(() => {
    if (protocolKey === '') {
      setFetchedRuntimeParameters(null)
      return
    }

    getProtocolInfoRos(activeProject, protocolName, (info) => {
      /* A failed fetch leaves the descriptors unknown. Treating the failure as "this
         protocol has no runtime parameters" would prune every stored value and leave the
         session startable with parameters the decider requires still missing. */
      setFetchedRuntimeParameters(info ? { key: protocolKey, infos: info.runtime_parameters ?? [] } : null)
    })
  }, [protocolKey, protocolList])

  /* Whether the descriptors in state describe the currently selected protocol. */
  const descriptorsReady = fetchedRuntimeParameters !== null && fetchedRuntimeParameters.key === protocolKey
  const runtimeParameterInfos = descriptorsReady ? fetchedRuntimeParameters.infos : []

  /* Reconcile the stored values against the descriptors: drop values whose descriptor no
     longer exists in the protocol (e.g. a parameter was removed or renamed on disk), and
     initialize any boolean parameter that has no value yet, since booleans default to false.

     This is keyed on the stored values as well as the descriptors, so it also runs when a
     session config is loaded from disk (e.g. after switching projects) rather than only when
     the protocol selection changes; otherwise a config that predates a parameter would keep
     that parameter unset until the user toggled it by hand. */
  const runtimeParametersKey = JSON.stringify(runtimeParameters)
  useEffect(() => {
    if (!descriptorsReady) {
      return
    }

    const validNames = new Set(runtimeParameterInfos.map((descriptor) => descriptor.name))
    const staleNames = Object.keys(runtimeParameters).filter((name) => !validNames.has(name))
    const uninitializedBooleans = runtimeParameterInfos.filter(
      (descriptor) => descriptor.type === 'bool' && runtimeParameters[descriptor.name] === undefined,
    )
    if (staleNames.length === 0 && uninitializedBooleans.length === 0) {
      return
    }

    const updated = { ...runtimeParameters }
    staleNames.forEach((name) => delete updated[name])
    uninitializedBooleans.forEach((descriptor) => {
      updated[descriptor.name] = false
    })
    setRuntimeParameters(updated)
  }, [fetchedRuntimeParameters, protocolKey, runtimeParametersKey])

  /* Every runtime parameter is required, so the session can only start once all of them have
     a usable value. Until the descriptors for the selected protocol are known, the parameters
     count as invalid: an empty descriptor list would otherwise look vacuously valid. */
  const runtimeParametersValid =
    protocolKey === '' ||
    (descriptorsReady &&
      runtimeParameterInfos.every((descriptor) => isRuntimeParameterSet(descriptor, runtimeParameters[descriptor.name])))

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
