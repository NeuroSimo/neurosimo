import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from 'roslib'

import { ros } from 'ros/ros'
import { useParameters } from './ParameterProvider'

interface TimingLatency extends ROSLIB.Message {
  latency: number
}

interface TimingError extends ROSLIB.Message {
  error: number
}

interface DecisionInfo extends ROSLIB.Message {
  sample_time: number
  stimulate: boolean
  decider_processing_duration: number
  preprocessor_processing_duration: number
  total_latency: number
}

export interface FilenameList extends ROSLIB.Message {
  filenames: string[]
}

export interface ExperimentState extends ROSLIB.Message {
  stage_name: string
  stage_index: number
  total_stages: number
  trial: number
  total_trials_in_stage: number
  ongoing: boolean
  in_rest: boolean
  paused: boolean
  experiment_time: number
  stage_start_time: number
  stage_elapsed_time: number
  rest_duration: number
  rest_elapsed: number
  rest_remaining: number
  next_stage_name: string
  next_is_rest: boolean
}

export interface LogMessage extends ROSLIB.Message {
  message: string
  sample_time: number
  level: number  // 0 = INFO, 1 = WARNING, 2 = ERROR
  is_initialization: boolean
}

export interface LogMessages extends ROSLIB.Message {
  messages: LogMessage[]
}

export const LogLevel = {
  INFO: 0,
  WARNING: 1,
  ERROR: 2,
} as const

interface PipelineContextType {
  preprocessorList: string[]
  preprocessorModule: string
  preprocessorEnabled: boolean
  preprocessorLogs: LogMessage[]

  deciderList: string[]
  deciderModule: string
  deciderEnabled: boolean
  deciderLogs: LogMessage[]

  presenterList: string[]
  presenterModule: string
  presenterEnabled: boolean
  presenterLogs: LogMessage[]

  protocolList: string[]
  protocolName: string

  timingLatency: TimingLatency | null
  timingError: TimingError | null
  decisionInfo: DecisionInfo | null
  experimentState: ExperimentState | null

  setTimingLatency: React.Dispatch<React.SetStateAction<TimingLatency | null>>
  setTimingError: React.Dispatch<React.SetStateAction<TimingError | null>>
  setDecisionInfo: React.Dispatch<React.SetStateAction<DecisionInfo | null>>
  setExperimentState: React.Dispatch<React.SetStateAction<ExperimentState | null>>
  clearAllLogs: () => void
}

const defaultPipelineState: PipelineContextType = {
  preprocessorList: [],
  preprocessorModule: '',
  preprocessorEnabled: false,
  preprocessorLogs: [],

  deciderList: [],
  deciderModule: '',
  deciderEnabled: false,
  deciderLogs: [],

  presenterList: [],
  presenterModule: '',
  presenterEnabled: false,
  presenterLogs: [],

  protocolList: [],
  protocolName: '',

  timingLatency: null,
  timingError: null,
  decisionInfo: null,
  experimentState: null,

  setTimingLatency: () => {
    console.warn('setTimingLatency is not yet initialized.')
  },
  setTimingError: () => {
    console.warn('setTimingError is not yet initialized.')
  },
  setDecisionInfo: () => {
    console.warn('setDecisionInfo is not yet initialized.')
  },
  setExperimentState: () => {
    console.warn('setExperimentState is not yet initialized.')
  },
  clearAllLogs: () => {
    console.warn('clearAllLogs is not yet initialized.')
  },
}

export const PipelineContext = React.createContext<PipelineContextType>(defaultPipelineState)

interface PipelineProviderProps {
  children: ReactNode
}

export const PipelineProvider: React.FC<PipelineProviderProps> = ({ children }) => {
  const { pipeline } = useParameters()

  const [preprocessorList, setPreprocessorList] = useState<string[]>([])
  const [preprocessorLogs, setPreprocessorLogs] = useState<LogMessage[]>([])

  const [deciderList, setDeciderList] = useState<string[]>([])
  const [deciderLogs, setDeciderLogs] = useState<LogMessage[]>([])

  const [presenterList, setPresenterList] = useState<string[]>([])
  const [presenterLogs, setPresenterLogs] = useState<LogMessage[]>([])

  const [protocolList, setProtocolList] = useState<string[]>([])

  // Get parameter values from structured parameter store
  const preprocessorModule = pipeline.preprocessor.module
  const preprocessorEnabled = pipeline.preprocessor.enabled
  const deciderModule = pipeline.decider.module
  const deciderEnabled = pipeline.decider.enabled
  const presenterModule = pipeline.presenter.module
  const presenterEnabled = pipeline.presenter.enabled
  const protocolName = pipeline.experiment.protocol

  const [timingLatency, setTimingLatency] = useState<TimingLatency | null>(null)
  const [timingError, setTimingError] = useState<TimingError | null>(null)
  const [decisionInfo, setDecisionInfo] = useState<DecisionInfo | null>(null)
  const [experimentState, setExperimentState] = useState<ExperimentState | null>(null)

  const clearAllLogs = () => {
    setPreprocessorLogs([])
    setDeciderLogs([])
    setPresenterLogs([])
  }

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

    /* Subscriber for timing latency. */
    const timingLatencySubscriber = new Topic<TimingLatency>({
      ros: ros,
      name: '/pipeline/timing/latency',
      messageType: 'pipeline_interfaces/TimingLatency',
    })

    timingLatencySubscriber.subscribe((message) => {
      setTimingLatency(message)
    })

    /* Subscriber for timing error. */
    const timingErrorSubscriber = new Topic<TimingError>({
      ros: ros,
      name: '/pipeline/timing/error',
      messageType: 'pipeline_interfaces/TimingError',
    })

    timingErrorSubscriber.subscribe((message) => {
      setTimingError(message)
    })

    /* Subscriber for decision info. */
    const decisionInfoSubscriber = new Topic<DecisionInfo>({
      ros: ros,
      name: '/pipeline/decision_info',
      messageType: 'pipeline_interfaces/DecisionInfo',
    })

    decisionInfoSubscriber.subscribe((message) => {
      setDecisionInfo(message)
    })

    /* Subscriber for preprocessor logs. */
    const preprocessorLogSubscriber = new Topic<LogMessages>({
      ros: ros,
      name: '/pipeline/preprocessor/log',
      messageType: 'pipeline_interfaces/LogMessages',
    })

    preprocessorLogSubscriber.subscribe((batch) => {
      // Unpack the batch of messages
      setPreprocessorLogs((prevLogs) => [...prevLogs, ...batch.messages])
    })

    /* Subscriber for decider logs. */
    const deciderLogSubscriber = new Topic<LogMessages>({
      ros: ros,
      name: '/pipeline/decider/log',
      messageType: 'pipeline_interfaces/LogMessages',
    })

    deciderLogSubscriber.subscribe((batch) => {
      // Unpack the batch of messages
      setDeciderLogs((prevLogs) => [...prevLogs, ...batch.messages])
    })

    /* Subscriber for presenter logs. */
    const presenterLogSubscriber = new Topic<LogMessages>({
      ros: ros,
      name: '/pipeline/presenter/log',
      messageType: 'pipeline_interfaces/LogMessages',
    })

    presenterLogSubscriber.subscribe((batch) => {
      // Unpack the batch of messages
      setPresenterLogs((prevLogs) => [...prevLogs, ...batch.messages])
    })

    /* Subscriber for experiment state. */
    const experimentStateSubscriber = new Topic<ExperimentState>({
      ros: ros,
      name: '/pipeline/experiment_state',
      messageType: 'pipeline_interfaces/ExperimentState',
    })

    experimentStateSubscriber.subscribe((message) => {
      setExperimentState(message)
    })

    /* Unsubscribers */
    return () => {
      preprocessorListSubscriber.unsubscribe()
      deciderListSubscriber.unsubscribe()
      presenterListSubscriber.unsubscribe()
      protocolListSubscriber.unsubscribe()

      timingLatencySubscriber.unsubscribe()
      timingErrorSubscriber.unsubscribe()
      decisionInfoSubscriber.unsubscribe()
      
      preprocessorLogSubscriber.unsubscribe()
      deciderLogSubscriber.unsubscribe()
      presenterLogSubscriber.unsubscribe()
      experimentStateSubscriber.unsubscribe()
    }
  }, [])

  return (
    <PipelineContext.Provider
      value={{
        preprocessorList,
        preprocessorModule,
        preprocessorEnabled,
        preprocessorLogs,
        deciderList,
        deciderModule,
        deciderEnabled,
        deciderLogs,
        presenterList,
        presenterModule,
        presenterEnabled,
        presenterLogs,
        protocolList,
        protocolName,
        timingLatency,
        timingError,
        decisionInfo,
        experimentState,
        setTimingLatency,
        setTimingError,
        setDecisionInfo,
        setExperimentState,
        clearAllLogs,
      }}
    >
      {children}
    </PipelineContext.Provider>
  )
}
