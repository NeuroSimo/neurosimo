import React, { useState, useEffect, ReactNode } from 'react'

import { useSessionConfig, RuntimeParameterValue } from './SessionConfigProvider'
import { useSystemConfig } from './SystemConfigProvider'
import { getProtocolInfoRos, RuntimeParameterInfo } from 'ros/experiment'
import { useProjectFileList } from 'utils/useProjectFileList'

/* A runtime parameter counts as "set" when it has a value the session can run with.
   A boolean is always set: an unticked checkbox is simply false. */
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
  const { pipeline, runtimeParameters } = useSessionConfig()
  const { activeProject } = useSystemConfig()

  const preprocessorList = useProjectFileList('/neurosimo/pipeline/preprocessor/list', activeProject)
  const deciderList = useProjectFileList('/neurosimo/pipeline/decider/list', activeProject)
  const presenterList = useProjectFileList('/neurosimo/pipeline/presenter/list', activeProject)
  const protocolList = useProjectFileList('/neurosimo/experiment/protocol/list', activeProject)

  /* Runtime parameter descriptors, tagged with the project/protocol they were fetched for,
     so that the inputs of a previously selected protocol are not rendered against the values
     of the current one while a fetch is in flight.

     null means the descriptors are not known: either nothing has been fetched yet, or the
     fetch failed. That is distinct from a successful fetch returning an empty list, which
     means the protocol declares no runtime parameters. */
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
      setFetchedRuntimeParameters(info ? { key: protocolKey, infos: info.runtime_parameters ?? [] } : null)
    })
  }, [protocolKey, protocolList])

  /* Whether the descriptors in state describe the currently selected protocol. */
  const descriptorsReady = fetchedRuntimeParameters !== null && fetchedRuntimeParameters.key === protocolKey
  const runtimeParameterInfos = descriptorsReady ? fetchedRuntimeParameters.infos : []

  /* Every runtime parameter is required, so the session can only start once all of them have
     a value. This mirrors the check the session manager makes when it compiles the session
     spec; it exists to keep the user from starting a session that would fail there. */
  const runtimeParametersValid = runtimeParameterInfos.every((descriptor) =>
    isRuntimeParameterSet(descriptor, runtimeParameters[descriptor.name]),
  )

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
