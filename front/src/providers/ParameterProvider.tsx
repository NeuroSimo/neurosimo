import React, { useState, useEffect, ReactNode, createContext, useContext } from 'react'
import { Topic } from 'roslib'
import { extractParameterValue } from '../ros/parameters'

import { ros } from 'ros/ros'

interface ParameterEvent extends ROSLIB.Message {
  timestamp: { sec: number; nanosec: number }
  node: string
  new_parameters: Array<{
    name: string
    value: {
      type: number
      bool_value?: boolean
      integer_value?: number
      double_value?: number
      string_value?: string
    }
  }>
  changed_parameters: Array<{
    name: string
    value: {
      type: number
      bool_value?: boolean
      integer_value?: number
      double_value?: number
      string_value?: string
    }
  }>
  deleted_parameters: Array<{
    name: string
  }>
}

// Structured parameter interfaces
interface MetadataParameters {
  subject_id: string
  notes: string
}

interface PipelineParameters {
  decider: {
    module: string
    enabled: boolean
  }
  preprocessor: {
    module: string
    enabled: boolean
  }
  presenter: {
    module: string
    enabled: boolean
  }
  experiment: {
    protocol: string
  }
}

interface SimulatorParameters {
  dataset_filename: string
  start_time: number
}

interface ParameterContextType {
  // Structured parameter access
  metadata: MetadataParameters
  pipeline: PipelineParameters
  simulator: SimulatorParameters

  // Convenience setters
  setSubjectId: (subjectId: string, callback?: () => void) => Promise<void>
  setNotes: (notes: string, callback?: () => void) => Promise<void>
  setDeciderModule: (module: string, callback?: () => void) => Promise<void>
  setDeciderEnabled: (enabled: boolean, callback?: () => void) => Promise<void>
  setPreprocessorModule: (module: string, callback?: () => void) => Promise<void>
  setPreprocessorEnabled: (enabled: boolean, callback?: () => void) => Promise<void>
  setPresenterModule: (module: string, callback?: () => void) => Promise<void>
  setPresenterEnabled: (enabled: boolean, callback?: () => void) => Promise<void>
  setExperimentProtocol: (protocol: string, callback?: () => void) => Promise<void>
  setSimulatorDataset: (filename: string, callback?: () => void) => Promise<void>
  setSimulatorStartTime: (startTime: number, callback?: () => void) => Promise<void>
  setPlaybackBagFilename: (bagFilename: string, callback?: () => void) => Promise<void>
  setPlaybackIsPreprocessed: (isPreprocessed: boolean, callback?: () => void) => Promise<void>
  setDataSource: (dataSource: string, callback?: () => void) => Promise<void>
}

// ESLint disable for intentionally empty functions used as defaults
/* eslint-disable @typescript-eslint/no-empty-function */
const asyncNoop = async () => {}
/* eslint-enable @typescript-eslint/no-empty-function */

const defaultParameterState: ParameterContextType = {
  metadata: {
    subject_id: '',
    notes: '',
  },
  pipeline: {
    decider: { module: '', enabled: false },
    preprocessor: { module: '', enabled: false },
    presenter: { module: '', enabled: false },
    experiment: { protocol: '' },
  },
  simulator: {
    dataset_filename: '',
    start_time: 0,
  },
  setSubjectId: asyncNoop,
  setNotes: asyncNoop,
  setDeciderModule: asyncNoop,
  setDeciderEnabled: asyncNoop,
  setPreprocessorModule: asyncNoop,
  setPreprocessorEnabled: asyncNoop,
  setPresenterModule: asyncNoop,
  setPresenterEnabled: asyncNoop,
  setExperimentProtocol: asyncNoop,
  setSimulatorDataset: asyncNoop,
  setSimulatorStartTime: asyncNoop,
  setPlaybackBagFilename: asyncNoop,
  setPlaybackIsPreprocessed: asyncNoop,
  setDataSource: asyncNoop,
}

export const ParameterContext = createContext<ParameterContextType>(defaultParameterState)

interface ParameterProviderProps {
  children: ReactNode
}

export const ParameterProvider: React.FC<ParameterProviderProps> = ({ children }) => {
  const [parameters, setParameters] = useState<Map<string, boolean | number | string>>(new Map())

  // Structured parameter access
  const metadata: MetadataParameters = {
    subject_id: (parameters.get('subject_id') as string) || '',
    notes: (parameters.get('notes') as string) || '',
  }

  const pipeline: PipelineParameters = {
    decider: {
      module: (parameters.get('decider.module') as string) || '',
      enabled: (parameters.get('decider.enabled') as boolean) || false,
    },
    preprocessor: {
      module: (parameters.get('preprocessor.module') as string) || '',
      enabled: (parameters.get('preprocessor.enabled') as boolean) || false,
    },
    presenter: {
      module: (parameters.get('presenter.module') as string) || '',
      enabled: (parameters.get('presenter.enabled') as boolean) || false,
    },
    experiment: {
      protocol: (parameters.get('experiment.protocol') as string) || '',
    },
  }

  const simulator: SimulatorParameters = {
    dataset_filename: (parameters.get('simulator.dataset_filename') as string) || '',
    start_time: (parameters.get('simulator.start_time') as number) || 0,
  }

  useEffect(() => {
    // Fetch initial parameter values on startup
    const fetchInitialParameters = async () => {
      try {
        const { getParametersRos } = await import('../ros/parameters')
        const parameterNames = [
          'subject_id', 'notes',
          'decider.module', 'decider.enabled',
          'preprocessor.module', 'preprocessor.enabled',
          'presenter.module', 'presenter.enabled',
          'experiment.protocol',
          'simulator.dataset_filename', 'simulator.start_time'
        ]

        const initialParams = await getParametersRos(parameterNames)
        setParameters(initialParams)
      } catch (error) {
        console.log('Failed to fetch initial parameters:', error)
        // Continue with empty parameters - the events will populate them as they change
      }
    }

    fetchInitialParameters()

    /* Subscriber for parameter events. */
    const parameterEventSubscriber = new Topic<ParameterEvent>({
      ros: ros,
      name: '/parameter_events',
      messageType: 'rcl_interfaces/ParameterEvent',
    })

    parameterEventSubscriber.subscribe((message) => {
      setParameters((prevParams) => {
        const newParams = new Map(prevParams)

        // Handle new parameters
        message.new_parameters.forEach((param) => {
          const value = extractParameterValue(param.value)
          if (value !== undefined) {
            newParams.set(param.name, value)
          }
        })

        // Handle changed parameters
        message.changed_parameters.forEach((param) => {
          const value = extractParameterValue(param.value)
          if (value !== undefined) {
            newParams.set(param.name, value)
          }
        })

        // Handle deleted parameters
        message.deleted_parameters.forEach((param) => {
          newParams.delete(param.name)
        })

        return newParams
      })
    })

    /* Cleanup */
    return () => {
      parameterEventSubscriber.unsubscribe()
    }
  }, [])

  // Convenience setters - use dynamic import to avoid circular dependencies
  const noop = () => {} // eslint-disable-line @typescript-eslint/no-empty-function

  const setSubjectId = async (subjectId: string, callback?: () => void): Promise<void> => {
    const { setParameterRos } = await import('../ros/parameters')
    setParameterRos('subject_id', subjectId, callback || noop)
  }
  const setNotes = async (notes: string, callback?: () => void): Promise<void> => {
    const { setParameterRos } = await import('../ros/parameters')
    setParameterRos('notes', notes, callback || noop)
  }
  const setDeciderModule = async (module: string, callback?: () => void): Promise<void> => {
    const { setParameterRos } = await import('../ros/parameters')
    setParameterRos('decider.module', module, callback || noop)
  }
  const setDeciderEnabled = async (enabled: boolean, callback?: () => void): Promise<void> => {
    const { setParameterRos } = await import('../ros/parameters')
    setParameterRos('decider.enabled', enabled, callback || noop)
  }
  const setPreprocessorModule = async (module: string, callback?: () => void): Promise<void> => {
    const { setParameterRos } = await import('../ros/parameters')
    setParameterRos('preprocessor.module', module, callback || noop)
  }
  const setPreprocessorEnabled = async (enabled: boolean, callback?: () => void): Promise<void> => {
    const { setParameterRos } = await import('../ros/parameters')
    setParameterRos('preprocessor.enabled', enabled, callback || noop)
  }
  const setPresenterModule = async (module: string, callback?: () => void): Promise<void> => {
    const { setParameterRos } = await import('../ros/parameters')
    setParameterRos('presenter.module', module, callback || noop)
  }
  const setPresenterEnabled = async (enabled: boolean, callback?: () => void): Promise<void> => {
    const { setParameterRos } = await import('../ros/parameters')
    setParameterRos('presenter.enabled', enabled, callback || noop)
  }
  const setExperimentProtocol = async (protocol: string, callback?: () => void): Promise<void> => {
    const { setParameterRos } = await import('../ros/parameters')
    setParameterRos('experiment.protocol', protocol, callback || noop)
  }
  const setSimulatorDataset = async (filename: string, callback?: () => void): Promise<void> => {
    const { setParameterRos } = await import('../ros/parameters')
    setParameterRos('simulator.dataset_filename', filename, callback || noop)
  }
  const setSimulatorStartTime = async (startTime: number, callback?: () => void): Promise<void> => {
    const { setParameterRos } = await import('../ros/parameters')
    setParameterRos('simulator.start_time', startTime, callback || noop)
  }
  const setPlaybackBagFilename = async (bagFilename: string, callback?: () => void): Promise<void> => {
    const { setParameterRos } = await import('../ros/parameters')
    setParameterRos('playback.bag_filename', bagFilename, callback || noop)
  }
  const setPlaybackIsPreprocessed = async (isPreprocessed: boolean, callback?: () => void): Promise<void> => {
    const { setParameterRos } = await import('../ros/parameters')
    setParameterRos('playback.is_preprocessed', isPreprocessed, callback || noop)
  }
  const setDataSource = async (dataSource: string, callback?: () => void): Promise<void> => {
    const { setParameterRos } = await import('../ros/parameters')
    setParameterRos('data_source', dataSource, callback || noop)
  }

  return (
    <ParameterContext.Provider
      value={{
        metadata,
        pipeline,
        simulator,
        setSubjectId,
        setNotes,
        setDeciderModule,
        setDeciderEnabled,
        setPreprocessorModule,
        setPreprocessorEnabled,
        setPresenterModule,
        setPresenterEnabled,
        setExperimentProtocol,
        setSimulatorDataset,
        setSimulatorStartTime,
        setPlaybackBagFilename,
        setPlaybackIsPreprocessed,
        setDataSource,
      }}
    >
      {children}
    </ParameterContext.Provider>
  )
}

export const useParameters = () => {
  const context = useContext(ParameterContext)
  if (!context) {
    throw new Error('useParameters must be used within a ParameterProvider')
  }
  return context
}
