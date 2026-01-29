import React, { useState, useEffect, ReactNode } from 'react'
import { Topic } from 'roslib'

import { ros } from 'ros/ros'

interface LoopbackLatency extends ROSLIB.Message {
  latency: number
}

interface DecisionTrace extends ROSLIB.Message {
  session_id: number[]
  decision_id: number
  status: number
  reference_sample_time: number
  reference_sample_index: number
  stimulate: boolean
  requested_stimulation_time: number
  decider_duration: number
  preprocessor_duration: number
  decision_path_latency: number
  system_time_decider_received: number
  system_time_decider_finished: number
  system_time_trigger_timer_received: number
  system_time_trigger_timer_finished: number
  system_time_hardware_fired: number
  sample_time_at_firing: number
  loopback_latency_at_firing: number
  latency_corrected_time_at_firing: number
  actual_stimulation_time: number
  actual_stimulation_sample_index: number
  timing_error: number
  pulse_confirmation_method: number
  pulse_confirmed: boolean
}

interface PipelineContextType {
  loopbackLatency: LoopbackLatency | null
  decisionTrace: DecisionTrace | null

  setLoopbackLatency: React.Dispatch<React.SetStateAction<LoopbackLatency | null>>
  setDecisionTrace: React.Dispatch<React.SetStateAction<DecisionTrace | null>>
}

const defaultPipelineState: PipelineContextType = {
  loopbackLatency: null,
  decisionTrace: null,

  setLoopbackLatency: () => {
    console.warn('setLoopbackLatency is not yet initialized.')
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
  const [loopbackLatency, setLoopbackLatency] = useState<LoopbackLatency | null>(null)
  const [decisionTrace, setDecisionTrace] = useState<DecisionTrace | null>(null)
  const [latencyTimeoutId, setLatencyTimeoutId] = useState<NodeJS.Timeout | null>(null)

  useEffect(() => {
    /* Subscriber for pipeline latency. */
    const loopbackLatencySubscriber = new Topic<LoopbackLatency>({
      ros: ros,
      name: '/pipeline/latency/trigger_loopback',
      messageType: 'pipeline_interfaces/LoopbackLatency',
    })

    loopbackLatencySubscriber.subscribe((message) => {
      console.log('loopbackLatency', message)
      setLoopbackLatency(message)

      // Clear existing timeout
      if (latencyTimeoutId) {
        clearTimeout(latencyTimeoutId)
      }

      // Set new timeout to clear latency after 0.5 seconds of no messages
      const timeoutId = setTimeout(() => {
        setLoopbackLatency(null)
      }, 500)
      setLatencyTimeoutId(timeoutId)
    })

    /* Subscriber for decision info. */
    const decisionTraceSubscriber = new Topic<DecisionTrace>({
      ros: ros,
      name: '/pipeline/decision_trace/final',
      messageType: 'pipeline_interfaces/DecisionTrace',
    })

    decisionTraceSubscriber.subscribe((message) => {
      console.log('decisionTrace', message)
      setDecisionTrace(message)
    })

    /* Unsubscribers */
    return () => {
      loopbackLatencySubscriber.unsubscribe()
      decisionTraceSubscriber.unsubscribe()
      if (latencyTimeoutId) {
        clearTimeout(latencyTimeoutId)
      }
    }
  }, [])

  return (
    <PipelineContext.Provider
      value={{
        loopbackLatency,
        decisionTrace,
        setLoopbackLatency,
        setDecisionTrace,
      }}
    >
      {children}
    </PipelineContext.Provider>
  )
}