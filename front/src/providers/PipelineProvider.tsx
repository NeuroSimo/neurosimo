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

interface PreprocessorList extends ROSLIB.Message {
  scripts: string[]
}

interface DeciderList extends ROSLIB.Message {
  scripts: string[]
}

interface PresenterList extends ROSLIB.Message {
  scripts: string[]
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
  is_initialization: boolean
}

interface PipelineContextType {
  preprocessorList: string[]
  preprocessorModule: string
  preprocessorEnabled: boolean

  deciderList: string[]
  deciderModule: string
  deciderEnabled: boolean
  deciderLogs: LogMessage[]

  presenterList: string[]
  presenterModule: string
  presenterEnabled: boolean

  timingLatency: TimingLatency | null
  timingError: TimingError | null
  decisionInfo: DecisionInfo | null

  setTimingLatency: React.Dispatch<React.SetStateAction<TimingLatency | null>>
  setTimingError: React.Dispatch<React.SetStateAction<TimingError | null>>
  setDecisionInfo: React.Dispatch<React.SetStateAction<DecisionInfo | null>>
  clearDeciderLogs: () => void
}

const defaultPipelineState: PipelineContextType = {
  preprocessorList: [],
  preprocessorModule: '',
  preprocessorEnabled: false,

  deciderList: [],
  deciderModule: '',
  deciderEnabled: false,
  deciderLogs: [],

  presenterList: [],
  presenterModule: '',
  presenterEnabled: false,

  timingLatency: null,
  timingError: null,
  decisionInfo: null,

  setTimingLatency: () => {
    console.warn('setTimingLatency is not yet initialized.')
  },
  setTimingError: () => {
    console.warn('setTimingError is not yet initialized.')
  },
  setDecisionInfo: () => {
    console.warn('setDecisionInfo is not yet initialized.')
  },
  clearDeciderLogs: () => {
    console.warn('clearDeciderLogs is not yet initialized.')
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

  const [deciderList, setDeciderList] = useState<string[]>([])
  const [deciderModule, setDeciderModule] = useState<string>('')
  const [deciderEnabled, setDeciderEnabled] = useState<boolean>(false)
  const [deciderLogs, setDeciderLogs] = useState<LogMessage[]>([])

  const [presenterList, setPresenterList] = useState<string[]>([])
  const [presenterModule, setPresenterModule] = useState<string>('')
  const [presenterEnabled, setPresenterEnabled] = useState<boolean>(false)

  const [timingLatency, setTimingLatency] = useState<TimingLatency | null>(null)
  const [timingError, setTimingError] = useState<TimingError | null>(null)
  const [decisionInfo, setDecisionInfo] = useState<DecisionInfo | null>(null)

  const clearDeciderLogs = () => {
    setDeciderLogs([])
  }

  useEffect(() => {
    /* Subscriber for preprocessor list. */
    const preprocessorListSubscriber = new Topic<PreprocessorList>({
      ros: ros,
      name: '/pipeline/preprocessor/list',
      messageType: 'project_interfaces/PreprocessorList',
    })

    preprocessorListSubscriber.subscribe((message) => {
      setPreprocessorList(message.scripts)
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
    const deciderListSubscriber = new Topic<DeciderList>({
      ros: ros,
      name: '/pipeline/decider/list',
      messageType: 'project_interfaces/DeciderList',
    })

    deciderListSubscriber.subscribe((message) => {
      setDeciderList(message.scripts)
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
    const presenterListSubscriber = new Topic<PresenterList>({
      ros: ros,
      name: '/pipeline/presenter/list',
      messageType: 'project_interfaces/PresenterList',
    })

    presenterListSubscriber.subscribe((message) => {
      setPresenterList(message.scripts)
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

    /* Subscriber for decider logs. */
    const deciderLogSubscriber = new Topic<LogMessage>({
      ros: ros,
      name: '/pipeline/decider/log',
      messageType: 'pipeline_interfaces/LogMessage',
    })

    deciderLogSubscriber.subscribe((message) => {
      setDeciderLogs((prevLogs) => [...prevLogs, message])
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

      timingLatencySubscriber.unsubscribe()
      timingErrorSubscriber.unsubscribe()
      decisionInfoSubscriber.unsubscribe()
      deciderLogSubscriber.unsubscribe()
    }
  }, [])

  return (
    <PipelineContext.Provider
      value={{
        preprocessorList,
        preprocessorModule,
        preprocessorEnabled,
        deciderList,
        deciderModule,
        deciderEnabled,
        deciderLogs,
        presenterList,
        presenterModule,
        presenterEnabled,
        timingLatency,
        timingError,
        decisionInfo,
        setTimingLatency,
        setTimingError,
        setDecisionInfo,
        clearDeciderLogs,
      }}
    >
      {children}
    </PipelineContext.Provider>
  )
}
