import React, { useState, useEffect, ReactNode } from 'react'
import { Topic } from 'roslib'

import { ros } from 'ros/ros'

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

interface ExperimentContextType {
  pipelineLatency: PipelineLatency | null
  decisionTrace: DecisionTrace | null
  experimentState: ExperimentState | null

  setPipelineLatency: React.Dispatch<React.SetStateAction<PipelineLatency | null>>
  setDecisionTrace: React.Dispatch<React.SetStateAction<DecisionTrace | null>>
  setExperimentState: React.Dispatch<React.SetStateAction<ExperimentState | null>>
}

const defaultExperimentState: ExperimentContextType = {
  pipelineLatency: null,
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
}

export const ExperimentContext = React.createContext<ExperimentContextType>(defaultExperimentState)

interface ExperimentProviderProps {
  children: ReactNode
}

export const ExperimentProvider: React.FC<ExperimentProviderProps> = ({ children }) => {
  const [pipelineLatency, setPipelineLatency] = useState<PipelineLatency | null>(null)
  const [decisionTrace, setDecisionTrace] = useState<DecisionTrace | null>(null)
  const [experimentState, setExperimentState] = useState<ExperimentState | null>(null)

  useEffect(() => {
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
      pipelineLatencySubscriber.unsubscribe()
      decisionTraceSubscriber.unsubscribe()
      experimentStateSubscriber.unsubscribe()
    }
  }, [])

  return (
    <ExperimentContext.Provider
      value={{
        pipelineLatency,
        decisionTrace,
        experimentState,
        setPipelineLatency,
        setDecisionTrace,
        setExperimentState,
      }}
    >
      {children}
    </ExperimentContext.Provider>
  )
}