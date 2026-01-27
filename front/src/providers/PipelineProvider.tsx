import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from 'roslib'

import { ros } from 'ros/ros'
import { useParameters } from './ParameterProvider'

interface PipelineLatency extends ROSLIB.Message {
  latency: number
}

interface DecisionTrace extends ROSLIB.Message {
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

  pipelineLatency: PipelineLatency | null
  decisionTrace: DecisionTrace | null
  experimentState: ExperimentState | null

  setPipelineLatency: React.Dispatch<React.SetStateAction<PipelineLatency | null>>
  setDecisionTrace: React.Dispatch<React.SetStateAction<DecisionTrace | null>>
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

  pipelineLatency: null,
  timingError: null,
  decisionTrace: null,
  experimentState: null,

  setPipelineLatency: () => {
    console.warn('setPipelineLatency is not yet initialized.')
  },
  setDecisionTrace: () => {
    console.warn('setDecisionTrace is not yet initialized.')
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

  const [pipelineLatency, setPipelineLatency] = useState<PipelineLatency | null>(null)
  const [decisionTrace, setDecisionTrace] = useState<DecisionTrace | null>(null)
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
    const pipelineLatencySubscriber = new Topic<PipelineLatency>({
      ros: ros,
      name: '/pipeline/latency',
      messageType: 'pipeline_interfaces/PipelineLatency',
    })

    pipelineLatencySubscriber.subscribe((message) => {
      setPipelineLatency(message)
    })

    /* Subscriber for decision info. */
    const decisionTraceSubscriber = new Topic<DecisionTrace>({
      ros: ros,
      name: '/pipeline/decision_info',
      messageType: 'pipeline_interfaces/DecisionTrace',
    })

    decisionTraceSubscriber.subscribe((message) => {
      setDecisionTrace(message)
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

      pipelineLatencySubscriber.unsubscribe()
      timingErrorSubscriber.unsubscribe()
      decisionTraceSubscriber.unsubscribe()
      
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
        pipelineLatency,
        decisionTrace,
        experimentState,
        setPipelineLatency,
        setDecisionTrace,
        setExperimentState,
        clearAllLogs,
      }}
    >
      {children}
    </PipelineContext.Provider>
  )
}
