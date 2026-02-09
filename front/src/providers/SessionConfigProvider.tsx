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

interface SessionConfigContextType {
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

const defaultSessionConfigState: SessionConfigContextType = {
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

export const SessionConfigContext = createContext<SessionConfigContextType>(defaultSessionConfigState)

interface SessionConfigProviderProps {
  children: ReactNode
}

export const SessionConfigProvider: React.FC<SessionConfigProviderProps> = ({ children }) => {
  const [sessionConfig, setSessionConfig] = useState<Map<string, boolean | number | string>>(new Map())

  // Structured parameter access
  const metadata: MetadataParameters = {
    subject_id: (sessionConfig.get('subject_id') as string) || '',
    notes: (sessionConfig.get('notes') as string) || '',
  }

  const pipeline: PipelineParameters = {
    decider: {
      module: (sessionConfig.get('decider.module') as string) || '',
      enabled: (sessionConfig.get('decider.enabled') as boolean) || false,
    },
    preprocessor: {
      module: (sessionConfig.get('preprocessor.module') as string) || '',
      enabled: (sessionConfig.get('preprocessor.enabled') as boolean) || false,
    },
    presenter: {
      module: (sessionConfig.get('presenter.module') as string) || '',
      enabled: (sessionConfig.get('presenter.enabled') as boolean) || false,
    },
    experiment: {
      protocol: (sessionConfig.get('experiment.protocol') as string) || '',
    },
  }

  const simulator: SimulatorParameters = {
    dataset_filename: (sessionConfig.get('simulator.dataset_filename') as string) || '',
    start_time: (sessionConfig.get('simulator.start_time') as number) || 0,
  }

  const dataSource = (sessionConfig.get('data_source') as string) || 'simulator'

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
      setSessionConfig((prevConfig) => {
        const newConfig = new Map(prevConfig)
        
        // Update all session config parameters
        newConfig.set('subject_id', msg.subject_id)
        newConfig.set('notes', msg.notes)
        newConfig.set('decider.module', msg.decider_module)
        newConfig.set('decider.enabled', msg.decider_enabled)
        newConfig.set('preprocessor.module', msg.preprocessor_module)
        newConfig.set('preprocessor.enabled', msg.preprocessor_enabled)
        newConfig.set('presenter.module', msg.presenter_module)
        newConfig.set('presenter.enabled', msg.presenter_enabled)
        newConfig.set('experiment.protocol', msg.protocol_filename)
        newConfig.set('simulator.dataset_filename', msg.simulator_dataset_filename)
        newConfig.set('simulator.start_time', msg.simulator_start_time)
        newConfig.set('data_source', msg.data_source)
        
        return newConfig
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
    <SessionConfigContext.Provider
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
    </SessionConfigContext.Provider>
  )
}

export const useSessionConfig = () => {
  const context = useContext(SessionConfigContext)
  if (!context) {
    throw new Error('useSessionConfig must be used within a SessionConfigProvider')
  }
  return context
}
