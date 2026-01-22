import React, { useState, useEffect, ReactNode, createContext, useContext } from 'react'
import { Topic } from 'roslib'

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
  pipeline: PipelineParameters
  simulator: SimulatorParameters

  // Convenience setters
  setDeciderModule: (module: string, callback?: () => void) => Promise<void>
  setDeciderEnabled: (enabled: boolean, callback?: () => void) => Promise<void>
  setPreprocessorModule: (module: string, callback?: () => void) => Promise<void>
  setPreprocessorEnabled: (enabled: boolean, callback?: () => void) => Promise<void>
  setPresenterModule: (module: string, callback?: () => void) => Promise<void>
  setPresenterEnabled: (enabled: boolean, callback?: () => void) => Promise<void>
  setExperimentProtocol: (protocol: string, callback?: () => void) => Promise<void>
  setSimulatorDataset: (filename: string, callback?: () => void) => Promise<void>
  setSimulatorStartTime: (startTime: number, callback?: () => void) => Promise<void>
}

// ESLint disable for intentionally empty functions used as defaults
/* eslint-disable @typescript-eslint/no-empty-function */
const asyncNoop = async () => {}
/* eslint-enable @typescript-eslint/no-empty-function */

const defaultParameterState: ParameterContextType = {
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
  setDeciderModule: asyncNoop,
  setDeciderEnabled: asyncNoop,
  setPreprocessorModule: asyncNoop,
  setPreprocessorEnabled: asyncNoop,
  setPresenterModule: asyncNoop,
  setPresenterEnabled: asyncNoop,
  setExperimentProtocol: asyncNoop,
  setSimulatorDataset: asyncNoop,
  setSimulatorStartTime: asyncNoop,
}

export const ParameterContext = createContext<ParameterContextType>(defaultParameterState)

interface ParameterProviderProps {
  children: ReactNode
}

export const ParameterProvider: React.FC<ParameterProviderProps> = ({ children }) => {
  const [parameters, setParameters] = useState<Map<string, boolean | number | string>>(new Map())

  // Structured parameter access
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

  return (
    <ParameterContext.Provider
      value={{
        pipeline,
        simulator,
        setDeciderModule,
        setDeciderEnabled,
        setPreprocessorModule,
        setPreprocessorEnabled,
        setPresenterModule,
        setPresenterEnabled,
        setExperimentProtocol,
        setSimulatorDataset,
        setSimulatorStartTime,
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

/**
 * Extract the actual value from a ROS2 ParameterValue structure
 */
const extractParameterValue = (paramValue: {
  type: number
  bool_value?: boolean
  integer_value?: number
  double_value?: number
  string_value?: string
}): boolean | number | string | undefined => {
  switch (paramValue.type) {
    case 1: // PARAMETER_BOOL
      return paramValue.bool_value
    case 2: // PARAMETER_INTEGER
      return paramValue.integer_value
    case 3: // PARAMETER_DOUBLE
      return paramValue.double_value
    case 4: // PARAMETER_STRING
      return paramValue.string_value
    default:
      console.warn(`Unknown parameter type: ${paramValue.type}`)
      return undefined
  }
}