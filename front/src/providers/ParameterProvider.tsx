import React, { useState, useEffect, ReactNode, createContext, useContext } from 'react'
import { Topic } from '@foxglove/roslibjs'

import { ros } from 'ros/ros'

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
  dataSource: string

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
  dataSource: 'simulator',
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

  const dataSource = (parameters.get('data_source') as string) || 'simulator'

  useEffect(() => {
    /* Subscriber for session config topic (latched). */
    const sessionConfigSubscriber = new Topic({
      ros: ros,
      name: '/session_configurator/config',
      messageType: 'system_interfaces/SessionConfig',
      queue_size: 1,
    })

    sessionConfigSubscriber.subscribe((message: ROSLIB.Message) => {
      const msg = message as any
      setParameters((prevParams) => {
        const newParams = new Map(prevParams)
        
        // Update all session config parameters
        newParams.set('subject_id', msg.subject_id)
        newParams.set('notes', msg.notes)
        newParams.set('decider.module', msg.decider_module)
        newParams.set('decider.enabled', msg.decider_enabled)
        newParams.set('preprocessor.module', msg.preprocessor_module)
        newParams.set('preprocessor.enabled', msg.preprocessor_enabled)
        newParams.set('presenter.module', msg.presenter_module)
        newParams.set('presenter.enabled', msg.presenter_enabled)
        newParams.set('experiment.protocol', msg.protocol_filename)
        newParams.set('simulator.dataset_filename', msg.simulator_dataset_filename)
        newParams.set('simulator.start_time', msg.simulator_start_time)
        newParams.set('data_source', msg.data_source)
        
        return newParams
      })
    })

    /* Cleanup */
    return () => {
      sessionConfigSubscriber.unsubscribe()
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
        dataSource,
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
