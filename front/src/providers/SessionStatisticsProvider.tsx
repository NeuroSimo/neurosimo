import React, { useState, useEffect, useRef, ReactNode } from 'react'
import { Topic } from '@foxglove/roslibjs'

import { ros } from 'ros/ros'

const STATUS_LABELS = {
  1: 'Decided no',
  2: 'Decided yes',
  3: 'Scheduled',
  4: 'Fired',
  5: 'Pulse observed',
  6: 'Missed',
  7: 'Loopback latency exceeded',
  8: 'Too late',
  9: 'Error'
} as const

export const getStatusLabel = (status: number): string => {
  return STATUS_LABELS[status as keyof typeof STATUS_LABELS] || 'Unknown'
}

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
  stimulation_horizon: number
  strict_stimulation_horizon: number
  decider_duration: number
  preprocessor_duration: number
  decision_path_latency: number
  system_time_decider_received: number
  system_time_decider_finished: number
  system_time_trigger_timer_received: number
  system_time_trigger_timer_finished: number
  system_time_hardware_fired: number
  loopback_latency_at_scheduling: number
  maximum_timing_offset: number
  maximum_loopback_latency: number
  trigger_to_pulse_delay: number
  latency_corrected_time_at_firing: number
  actual_stimulation_time: number
  actual_stimulation_sample_index: number
  timing_error: number
  pulse_confirmation_method: number
  pulse_confirmed: boolean
}

interface SessionStatisticsContextType {
  loopbackLatency: LoopbackLatency | null
  decisionTrace: DecisionTrace | null

  setLoopbackLatency: React.Dispatch<React.SetStateAction<LoopbackLatency | null>>
  setDecisionTrace: React.Dispatch<React.SetStateAction<DecisionTrace | null>>
}

const defaultSessionStatisticsState: SessionStatisticsContextType = {
  loopbackLatency: null,
  decisionTrace: null,

  setLoopbackLatency: () => {
    console.warn('setLoopbackLatency is not yet initialized.')
  },
  setDecisionTrace: () => {
    console.warn('setDecisionTrace is not yet initialized.')
  },
}

export const SessionStatisticsContext = React.createContext<SessionStatisticsContextType>(defaultSessionStatisticsState)

interface SessionStatisticsProviderProps {
  children: ReactNode
}

export const SessionStatisticsProvider: React.FC<SessionStatisticsProviderProps> = ({ children }) => {
  const [loopbackLatency, setLoopbackLatency] = useState<LoopbackLatency | null>(null)
  const [decisionTrace, setDecisionTrace] = useState<DecisionTrace | null>(null)
  const latencyTimeoutRef = useRef<NodeJS.Timeout | null>(null)

  useEffect(() => {
    /* Subscriber for loopback latency. */
    const loopbackLatencySubscriber = new Topic<LoopbackLatency>({
      ros: ros,
      name: '/pipeline/latency/trigger_loopback',
      messageType: 'pipeline_interfaces/LoopbackLatency',
    })

    loopbackLatencySubscriber.subscribe((message) => {
      console.log('loopbackLatency', message)
      setLoopbackLatency(message)

      // Clear existing timeout
      if (latencyTimeoutRef.current) {
        clearTimeout(latencyTimeoutRef.current)
      }

      // Set new timeout to clear latency after 0.5 seconds of no messages
      latencyTimeoutRef.current = setTimeout(() => {
        setLoopbackLatency(null)
      }, 500)
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
      if (latencyTimeoutRef.current) {
        clearTimeout(latencyTimeoutRef.current)
      }
    }
  }, [])

  return (
    <SessionStatisticsContext.Provider
      value={{
        loopbackLatency,
        decisionTrace,
        setLoopbackLatency,
        setDecisionTrace,
      }}
    >
      {children}
    </SessionStatisticsContext.Provider>
  )
}
