import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from 'roslib'

import { ros } from 'ros/ros'

interface TimingLatency extends ROSLIB.Message {
  latency: number
}

interface TimingError extends ROSLIB.Message {
  error: number
}

interface DecisionInfo extends ROSLIB.Message {
  stimulate: boolean
  feasible: boolean
  decision_time: number
  decider_latency: number
  preprocessor_latency: number
  total_latency: number
}

interface ModuleList extends ROSLIB.Message {
  modules: string[]
}

interface ProtocolList extends ROSLIB.Message {
  protocols: string[]
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

interface RosBoolean extends ROSLIB.Message {
  data: boolean
}

interface RosString extends ROSLIB.Message {
  data: string
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
  const [preprocessorList, setPreprocessorList] = useState<string[]>([])
  const [preprocessorModule, setPreprocessorModule] = useState<string>('')
  const [preprocessorEnabled, setPreprocessorEnabled] = useState<boolean>(false)
  const [preprocessorLogs, setPreprocessorLogs] = useState<LogMessage[]>([])

  const [deciderList, setDeciderList] = useState<string[]>([])
  const [deciderModule, setDeciderModule] = useState<string>('')
  const [deciderEnabled, setDeciderEnabled] = useState<boolean>(false)
  const [deciderLogs, setDeciderLogs] = useState<LogMessage[]>([])

  const [presenterList, setPresenterList] = useState<string[]>([])
  const [presenterModule, setPresenterModule] = useState<string>('')
  const [presenterEnabled, setPresenterEnabled] = useState<boolean>(false)
  const [presenterLogs, setPresenterLogs] = useState<LogMessage[]>([])

  const [protocolList, setProtocolList] = useState<string[]>([])
  const [protocolName, setProtocolName] = useState<string>('')

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
    const preprocessorListSubscriber = new Topic<ModuleList>({
      ros: ros,
      name: '/pipeline/preprocessor/list',
      messageType: 'project_interfaces/ModuleList',
    })

    preprocessorListSubscriber.subscribe((message) => {
      setPreprocessorList(message.modules)
    })

    /* Subscriber for preprocessor module. */
    const preprocessorModuleSubscriber = new Topic<RosString>({
      ros: ros,
      name: '/pipeline/preprocessor/module',
      messageType: 'std_msgs/String',
    })

    preprocessorModuleSubscriber.subscribe((message) => {
      setPreprocessorModule(message.data)
    })

    /* Subscriber for preprocessor enabled. */
    const preprocessorEnabledSubscriber = new Topic<RosBoolean>({
      ros: ros,
      name: '/pipeline/preprocessor/enabled',
      messageType: 'std_msgs/Bool',
    })

    preprocessorEnabledSubscriber.subscribe((message) => {
      setPreprocessorEnabled(message.data)
    })

    /* Subscriber for decider list. */
    const deciderListSubscriber = new Topic<ModuleList>({
      ros: ros,
      name: '/pipeline/decider/list',
      messageType: 'project_interfaces/ModuleList',
    })

    deciderListSubscriber.subscribe((message) => {
      setDeciderList(message.modules)
    })

    /* Subscriber for decider module. */
    const deciderModuleSubscriber = new Topic<RosString>({
      ros: ros,
      name: '/pipeline/decider/module',
      messageType: 'std_msgs/String',
    })

    deciderModuleSubscriber.subscribe((message) => {
      setDeciderModule(message.data)
    })

    /* Subscriber for decider enabled. */
    const deciderEnabledSubscriber = new Topic<RosBoolean>({
      ros: ros,
      name: '/pipeline/decider/enabled',
      messageType: 'std_msgs/Bool',
    })

    deciderEnabledSubscriber.subscribe((message) => {
      setDeciderEnabled(message.data)
    })

    /* Subscriber for presenter list. */
    const presenterListSubscriber = new Topic<ModuleList>({
      ros: ros,
      name: '/pipeline/presenter/list',
      messageType: 'project_interfaces/ModuleList',
    })

    presenterListSubscriber.subscribe((message) => {
      setPresenterList(message.modules)
    })

    /* Subscriber for presenter module. */
    const presenterModuleSubscriber = new Topic<RosString>({
      ros: ros,
      name: '/pipeline/presenter/module',
      messageType: 'std_msgs/String',
    })

    presenterModuleSubscriber.subscribe((message) => {
      setPresenterModule(message.data)
    })

    /* Subscriber for presenter enabled. */
    const presenterEnabledSubscriber = new Topic<RosBoolean>({
      ros: ros,
      name: '/pipeline/presenter/enabled',
      messageType: 'std_msgs/Bool',
    })

    presenterEnabledSubscriber.subscribe((message) => {
      setPresenterEnabled(message.data)
    })

    /* Subscriber for available protocols. */
    const protocolListSubscriber = new Topic<ProtocolList>({
      ros: ros,
      name: '/experiment/protocol/list',
      messageType: 'project_interfaces/ProtocolList',
    })

    protocolListSubscriber.subscribe((message) => {
      setProtocolList(message.protocols)
    })

    /* Subscriber for active protocol. */
    const protocolSubscriber = new Topic<RosString>({
      ros: ros,
      name: '/experiment/protocol',
      messageType: 'std_msgs/String',
    })

    protocolSubscriber.subscribe((message) => {
      setProtocolName(message.data)
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
      preprocessorModuleSubscriber.unsubscribe()
      preprocessorEnabledSubscriber.unsubscribe()

      deciderListSubscriber.unsubscribe()
      deciderModuleSubscriber.unsubscribe()
      deciderEnabledSubscriber.unsubscribe()

      presenterListSubscriber.unsubscribe()
      presenterModuleSubscriber.unsubscribe()
      presenterEnabledSubscriber.unsubscribe()

      protocolListSubscriber.unsubscribe()
      protocolSubscriber.unsubscribe()

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
