import React, { useState, useEffect, useRef, ReactNode } from 'react'
import { Topic } from '@foxglove/roslibjs'

import { ros } from 'ros/ros'

const DECISION_STATUS_LABELS = {
  1: 'Decided no',
  2: 'Decided yes',
} as const

const TRIAL_STATUS_LABELS = {
  1: 'Scheduled',
  2: 'Fired',
  3: 'Pulse observed',
  4: 'Missed',
  5: 'Loopback latency exceeded',
  6: 'Too late',
  7: 'Error',
} as const

export const getStatusLabel = (status: number): string => {
  return (
    DECISION_STATUS_LABELS[status as keyof typeof DECISION_STATUS_LABELS] ||
    TRIAL_STATUS_LABELS[status as keyof typeof TRIAL_STATUS_LABELS] ||
    'Unknown'
  )
}

interface Latency extends ROSLIB.Message {
  latency: number
}

export interface DecisionTrace extends ROSLIB.Message {
  status: number
  decision_id: number
  reference_sample_time: number
  reference_sample_index: number
  stimulate: boolean
  decider_duration: number
  preprocessor_duration: number
  decision_path_latency: number
  system_time_decider_received: number
  system_time_decider_finished: number
}

export interface AttemptTrace extends ROSLIB.Message {
  session_id: number[]
  attempt_in_session: number
  status: number
  requested_stimulation_time: number
  stimulation_horizon: number
  strict_stimulation_horizon: number
  maximum_timing_offset: number
  maximum_loopback_latency: number
  trigger_to_pulse_delay: number
  system_time_trigger_timer_received: number
  system_time_trigger_timer_finished: number
  system_time_hardware_fired: number
  loopback_latency_at_scheduling: number
  latency_corrected_time_at_firing: number
  actual_stimulation_time: number
  actual_stimulation_sample_index: number
  timing_offset: number
  decision: DecisionTrace
}

interface SessionStatisticsContextType {
  loopbackLatency: Latency | null
  pulseProcessingLatency: Latency | null
  eventProcessingLatency: Latency | null
  decisionTrace: DecisionTrace | null
  attemptTrace: AttemptTrace | null

  setLoopbackLatency: React.Dispatch<React.SetStateAction<Latency | null>>
  setPulseProcessingLatency: React.Dispatch<React.SetStateAction<Latency | null>>
  setEventProcessingLatency: React.Dispatch<React.SetStateAction<Latency | null>>
  setDecisionTrace: React.Dispatch<React.SetStateAction<DecisionTrace | null>>
  setAttemptTrace: React.Dispatch<React.SetStateAction<AttemptTrace | null>>
}

const defaultSessionStatisticsState: SessionStatisticsContextType = {
  loopbackLatency: null,
  pulseProcessingLatency: null,
  eventProcessingLatency: null,
  decisionTrace: null,
  attemptTrace: null,

  setLoopbackLatency: () => {
    console.warn('setLoopbackLatency is not yet initialized.')
  },
  setPulseProcessingLatency: () => {
    console.warn('setPulseProcessingLatency is not yet initialized.')
  },
  setEventProcessingLatency: () => {
    console.warn('setEventProcessingLatency is not yet initialized.')
  },
  setDecisionTrace: () => {
    console.warn('setDecisionTrace is not yet initialized.')
  },
  setAttemptTrace: () => {
    console.warn('setAttemptTrace is not yet initialized.')
  },
}

export const SessionStatisticsContext = React.createContext<SessionStatisticsContextType>(defaultSessionStatisticsState)

interface SessionStatisticsProviderProps {
  children: ReactNode
}

export const SessionStatisticsProvider: React.FC<SessionStatisticsProviderProps> = ({ children }) => {
  const [loopbackLatency, setLoopbackLatency] = useState<Latency | null>(null)
  const [pulseProcessingLatency, setPulseProcessingLatency] = useState<Latency | null>(null)
  const [eventProcessingLatency, setEventProcessingLatency] = useState<Latency | null>(null)
  const [decisionTrace, setDecisionTrace] = useState<DecisionTrace | null>(null)
  const [attemptTrace, setAttemptTrace] = useState<AttemptTrace | null>(null)
  const loopbackLatencyTimeoutRef = useRef<NodeJS.Timeout | null>(null)

  useEffect(() => {
    /* Subscriber for loopback latency. */
    const loopbackLatencySubscriber = new Topic<Latency>({
      ros: ros,
      name: '/neurosimo/pipeline/latency/loopback',
      messageType: 'neurosimo_pipeline_interfaces/Latency',
    })

    loopbackLatencySubscriber.subscribe((message) => {
      setLoopbackLatency(message)

      // Clear existing timeout
      if (loopbackLatencyTimeoutRef.current) {
        clearTimeout(loopbackLatencyTimeoutRef.current)
      }

      // Set new timeout to clear latency after 0.5 seconds of no messages
      loopbackLatencyTimeoutRef.current = setTimeout(() => {
        setLoopbackLatency(null)
      }, 500)
    })

    /* Subscriber for pulse processing latency. */
    const pulseProcessingLatencySubscriber = new Topic<Latency>({
      ros: ros,
      name: '/neurosimo/pipeline/latency/pulse_processing',
      messageType: 'neurosimo_pipeline_interfaces/Latency',
    })

    pulseProcessingLatencySubscriber.subscribe((message) => {
      console.log('pulseProcessingLatency', message)
      setPulseProcessingLatency(message)
    })

    /* Subscriber for event processing latency. */
    const eventProcessingLatencySubscriber = new Topic<Latency>({
      ros: ros,
      name: '/neurosimo/pipeline/latency/event_processing',
      messageType: 'neurosimo_pipeline_interfaces/Latency',
    })

    eventProcessingLatencySubscriber.subscribe((message) => {
      console.log('eventProcessingLatency', message)
      setEventProcessingLatency(message)
    })

    /* Subscriber for decision traces (all decisions from Decider). */
    const decisionTraceSubscriber = new Topic<DecisionTrace>({
      ros: ros,
      name: '/neurosimo/pipeline/decision_trace',
      messageType: 'neurosimo_pipeline_interfaces/DecisionTrace',
    })

    decisionTraceSubscriber.subscribe((message) => {
      setDecisionTrace(message)
    })

    /* Subscriber for final attempt traces. */
    const attemptTraceSubscriber = new Topic<AttemptTrace>({
      ros: ros,
      name: '/neurosimo/pipeline/attempt_trace/final',
      messageType: 'neurosimo_pipeline_interfaces/AttemptTrace',
    })

    attemptTraceSubscriber.subscribe((message) => {
      console.log('attemptTrace', message)
      setAttemptTrace(message)
    })

    /* Unsubscribers */
    return () => {
      loopbackLatencySubscriber.unsubscribe()
      pulseProcessingLatencySubscriber.unsubscribe()
      eventProcessingLatencySubscriber.unsubscribe()
      decisionTraceSubscriber.unsubscribe()
      attemptTraceSubscriber.unsubscribe()
      if (loopbackLatencyTimeoutRef.current) {
        clearTimeout(loopbackLatencyTimeoutRef.current)
      }
    }
  }, [])

  return (
    <SessionStatisticsContext.Provider
      value={{
        loopbackLatency,
        pulseProcessingLatency,
        eventProcessingLatency,
        decisionTrace,
        attemptTrace,
        setLoopbackLatency,
        setPulseProcessingLatency,
        setEventProcessingLatency,
        setDecisionTrace,
        setAttemptTrace,
      }}
    >
      {children}
    </SessionStatisticsContext.Provider>
  )
}
