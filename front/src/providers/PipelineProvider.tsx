import React, { useState, useEffect, ReactNode } from 'react'
import { Topic } from 'roslib'

import { ros } from 'ros/ros'

interface TriggerLoopbackLatency extends ROSLIB.Message {
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
  system_time_decider_received: number
  system_time_decider_finished: number
  system_time_trigger_timer_received: number
  system_time_trigger_timer_finished: number
  system_time_hardware_fired: number
  sample_time_at_firing: number
  trigger_loopback_latency_at_firing: number
  latency_corrected_time_at_firing: number
  actual_stimulation_time: number
  actual_stimulation_sample_index: number
  timing_error: number
  pulse_confirmation_method: number
  pulse_confirmed: boolean
}

interface PipelineContextType {
  triggerLoopbackLatency: TriggerLoopbackLatency | null
  decisionTrace: DecisionTrace | null

  setTriggerLoopbackLatency: React.Dispatch<React.SetStateAction<TriggerLoopbackLatency | null>>
  setDecisionTrace: React.Dispatch<React.SetStateAction<DecisionTrace | null>>
}

const defaultPipelineState: PipelineContextType = {
  triggerLoopbackLatency: null,
  decisionTrace: null,

  setTriggerLoopbackLatency: () => {
    console.warn('setTriggerLoopbackLatency is not yet initialized.')
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
  const [triggerLoopbackLatency, setTriggerLoopbackLatency] = useState<TriggerLoopbackLatency | null>(null)
  const [decisionTrace, setDecisionTrace] = useState<DecisionTrace | null>(null)

  useEffect(() => {
    /* Subscriber for pipeline latency. */
    const triggerLoopbackLatencySubscriber = new Topic<TriggerLoopbackLatency>({
      ros: ros,
      name: '/pipeline/latency/trigger_loopback',
      messageType: 'pipeline_interfaces/TriggerLoopbackLatency',
    })

    triggerLoopbackLatencySubscriber.subscribe((message) => {
      console.log('triggerLoopbackLatency', message)
      setTriggerLoopbackLatency(message)
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
      triggerLoopbackLatencySubscriber.unsubscribe()
      decisionTraceSubscriber.unsubscribe()
    }
  }, [])

  return (
    <PipelineContext.Provider
      value={{
        triggerLoopbackLatency,
        decisionTrace,
        setTriggerLoopbackLatency,
        setDecisionTrace,
      }}
    >
      {children}
    </PipelineContext.Provider>
  )
}