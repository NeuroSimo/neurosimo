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

interface Latency extends ROSLIB.Message {
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
  timing_offset: number
  pulse_confirmation_method: number
  pulse_confirmed: boolean
}

interface SessionStatisticsContextType {
  loopbackLatency: Latency | null
  pulseProcessingLatency: Latency | null
  eventProcessingLatency: Latency | null
  decisionTrace: DecisionTrace | null

  setLoopbackLatency: React.Dispatch<React.SetStateAction<Latency | null>>
  setPulseProcessingLatency: React.Dispatch<React.SetStateAction<Latency | null>>
  setEventProcessingLatency: React.Dispatch<React.SetStateAction<Latency | null>>
  setDecisionTrace: React.Dispatch<React.SetStateAction<DecisionTrace | null>>
}

const defaultSessionStatisticsState: SessionStatisticsContextType = {
  loopbackLatency: null,
  pulseProcessingLatency: null,
  eventProcessingLatency: null,
  decisionTrace: null,

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
  const loopbackLatencyTimeoutRef = useRef<NodeJS.Timeout | null>(null)

  useEffect(() => {
    /* Subscriber for loopback latency. */
    const loopbackLatencySubscriber = new Topic<Latency>({
      ros: ros,
      name: '/pipeline/latency/loopback',
      messageType: 'pipeline_interfaces/LoopbackLatency',
    })

    loopbackLatencySubscriber.subscribe((message) => {
      console.log('loopbackLatency', message)
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
      name: '/pipeline/latency/pulse_processing',
      messageType: 'pipeline_interfaces/Latency',
    })

    pulseProcessingLatencySubscriber.subscribe((message) => {
      console.log('pulseProcessingLatency', message)
      setPulseProcessingLatency(message)
    })

    /* Subscriber for event processing latency. */
    const eventProcessingLatencySubscriber = new Topic<Latency>({
      ros: ros,
      name: '/pipeline/latency/event_processing',
      messageType: 'pipeline_interfaces/Latency',
    })

    eventProcessingLatencySubscriber.subscribe((message) => {
      console.log('eventProcessingLatency', message)
      setEventProcessingLatency(message)
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
      pulseProcessingLatencySubscriber.unsubscribe()
      eventProcessingLatencySubscriber.unsubscribe()
      decisionTraceSubscriber.unsubscribe()
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
        setLoopbackLatency,
        setPulseProcessingLatency,
        setEventProcessingLatency,
        setDecisionTrace,
      }}
    >
      {children}
    </SessionStatisticsContext.Provider>
  )
}
