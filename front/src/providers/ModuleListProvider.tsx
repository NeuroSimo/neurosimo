import React, { useState, useEffect, ReactNode } from 'react'

import { useSessionConfig, RuntimeParameterValue, RuntimeParameters } from './SessionConfigProvider'
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

  /* The runtime parameters currently on display: the protocol they belong to, their
     descriptors, and the values entered for them. The protocol is not necessarily the
     selected one; see the pinning in the provider below. */
  runtimeParameterProtocol: string
  runtimeParameterInfos: RuntimeParameterInfo[]
  runtimeParameterValues: RuntimeParameters

  /* Whether every runtime parameter currently has a usable value (all are required). */
  runtimeParametersValid: boolean

  /* Names of the runtime parameters that are still missing a value, and whether they
     should currently be pointed out to the user (set when a session start is attempted
     while some of them are unset). */
  missingRuntimeParameters: string[]
  showMissingRuntimeParameters: boolean
  flagMissingRuntimeParameters: () => void
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

  runtimeParameterProtocol: '',
  runtimeParameterInfos: [],
  runtimeParameterValues: {},
  runtimeParametersValid: true,

  missingRuntimeParameters: [],
  showMissingRuntimeParameters: false,
  flagMissingRuntimeParameters: () => undefined,
}

export const ModuleListContext = React.createContext<ModuleListContextType>(defaultModuleListState)

interface ModuleListProviderProps {
  children: ReactNode
}

export const ModuleListProvider: React.FC<ModuleListProviderProps> = ({ children }) => {
  const { pipeline, getRuntimeParameters } = useSessionConfig()
  const { activeProject } = useSystemConfig()

  const preprocessorList = useProjectFileList('/neurosimo/pipeline/preprocessor/list', activeProject)
  const deciderList = useProjectFileList('/neurosimo/pipeline/decider/list', activeProject)
  const presenterList = useProjectFileList('/neurosimo/pipeline/presenter/list', activeProject)
  const protocolList = useProjectFileList('/neurosimo/experiment/protocol/list', activeProject)

  /* Runtime parameter descriptors, tagged with the protocol they were fetched for,
     so that the values they are rendered against can be read for that same protocol.

     null means the descriptors are not known: either nothing has been fetched yet, or the
     fetch failed. That is distinct from a successful fetch returning an empty list, which
     means the protocol declares no runtime parameters. */
  const [fetchedRuntimeParameters, setFetchedRuntimeParameters] = useState<{
    protocol: string
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

    /* Responses can arrive out of order when protocols are switched in quick succession;
       drop the ones belonging to a selection that has already been superseded. */
    let superseded = false

    getProtocolInfoRos(activeProject, protocolName, (info) => {
      if (superseded) {
        return
      }
      setFetchedRuntimeParameters(
        info ? { protocol: protocolName, infos: info.runtime_parameters ?? [] } : null,
      )
    })

    return () => {
      superseded = true
    }
  }, [protocolKey, protocolList])

  /* The runtime parameters of the previously selected protocol stay on display until the
     descriptors of the newly selected one have arrived, so that switching protocols does not
     flash an empty parameter list. Descriptors and values are both taken for the protocol the
     descriptors were fetched for, so what is shown stays consistent across the swap. */
  const runtimeParameterProtocol = fetchedRuntimeParameters?.protocol ?? ''
  const runtimeParameterInfos = fetchedRuntimeParameters?.infos ?? []
  const runtimeParameterValues = getRuntimeParameters(runtimeParameterProtocol)

  /* Every runtime parameter is required, so the session can only start once all of them have
     a value. This mirrors the check the session manager makes when it compiles the session
     spec; it exists to keep the user from starting a session that would fail there. */
  const missingRuntimeParameters = runtimeParameterInfos
    .filter((descriptor) => !isRuntimeParameterSet(descriptor, runtimeParameterValues[descriptor.name]))
    .map((descriptor) => descriptor.name)

  const runtimeParametersValid = missingRuntimeParameters.length === 0

  const [showMissingRuntimeParameters, setShowMissingRuntimeParameters] = useState(false)

  /* Stop pointing out the missing parameters once they have all been filled in. */
  useEffect(() => {
    if (runtimeParametersValid) {
      setShowMissingRuntimeParameters(false)
    }
  }, [runtimeParametersValid])

  const flagMissingRuntimeParameters = () => setShowMissingRuntimeParameters(true)

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
        runtimeParameterProtocol,
        runtimeParameterInfos,
        runtimeParameterValues,
        runtimeParametersValid,
        missingRuntimeParameters,
        showMissingRuntimeParameters,
        flagMissingRuntimeParameters,
      }}
    >
      {children}
    </ModuleListContext.Provider>
  )
}
