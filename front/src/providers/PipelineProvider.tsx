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

interface PipelineContextType {
  pipelineLatency: PipelineLatency | null
  decisionTrace: DecisionTrace | null

  setPipelineLatency: React.Dispatch<React.SetStateAction<PipelineLatency | null>>
  setDecisionTrace: React.Dispatch<React.SetStateAction<DecisionTrace | null>>
}

const defaultPipelineState: PipelineContextType = {
  pipelineLatency: null,
  decisionTrace: null,

  setPipelineLatency: () => {
    console.warn('setPipelineLatency is not yet initialized.')
  },
  setDecisionTrace: () => {
    console.warn('setDecisionTrace is not yet initialized.')
  },
}

export const PipelineContext = React.createContext<PipelineContextType>(defaultPipelineState)

interface PipelineProviderProps {
  children: ReactNode
}

export const PipelineProvider: React.FC<PipelineProviderProps> = ({ children }) => {
  const [pipelineLatency, setPipelineLatency] = useState<PipelineLatency | null>(null)
  const [decisionTrace, setDecisionTrace] = useState<DecisionTrace | null>(null)

  useEffect(() => {
    /* Subscriber for pipeline latency. */
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

    /* Unsubscribers */
    return () => {
      pipelineLatencySubscriber.unsubscribe()
      decisionTraceSubscriber.unsubscribe()
    }
  }, [])

  return (
    <PipelineContext.Provider
      value={{
        pipelineLatency,
        decisionTrace,
        setPipelineLatency,
        setDecisionTrace,
      }}
    >
      {children}
    </PipelineContext.Provider>
  )
}