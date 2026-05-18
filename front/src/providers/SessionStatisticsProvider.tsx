import React, { useState, useEffect, ReactNode } from 'react'
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
  4: 'Loopback latency exceeded',
  5: 'Too late',
  6: 'Error',
} as const

export const getStatusLabel = (status: number): string => {
  return (
    DECISION_STATUS_LABELS[status as keyof typeof DECISION_STATUS_LABELS] ||
    TRIAL_STATUS_LABELS[status as keyof typeof TRIAL_STATUS_LABELS] ||
    'Unknown'
  )
}

interface Float64 extends ROSLIB.Message {
  data: number
}

export interface DecisionTrace extends ROSLIB.Message {
  status: number
  decision_id: number
  reference_sample_time: number
  reference_sample_index: number
  stimulate: boolean
  eeg_device_processing_duration: number
  decider_duration: number
  preprocessor_duration: number
  overhead_duration: number
  total_duration: number
  system_time_decider_received: number
  system_time_decider_finished: number
}

export interface AttemptTrace extends ROSLIB.Message {
  session_id: number[]
  attempt_in_session: number
  status: number
  attempt_timing: number
  attempt_type: string
  decision_id: number
  requested_stimulation_time: number
  reference_time: number
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
  invalid_trial: boolean
}

interface SessionStatisticsContextType {
  pulseProcessingTime: number | null
  eventProcessingTime: number | null
  decisionTrace: DecisionTrace | null
  attemptTrace: AttemptTrace | null

  setPulseProcessingTime: React.Dispatch<React.SetStateAction<number | null>>
  setEventProcessingTime: React.Dispatch<React.SetStateAction<number | null>>
  setDecisionTrace: React.Dispatch<React.SetStateAction<DecisionTrace | null>>
  setAttemptTrace: React.Dispatch<React.SetStateAction<AttemptTrace | null>>
}

const defaultSessionStatisticsState: SessionStatisticsContextType = {
  pulseProcessingTime: null,
  eventProcessingTime: null,
  decisionTrace: null,
  attemptTrace: null,

  setPulseProcessingTime: () => {
    console.warn('setPulseProcessingTime is not yet initialized.')
  },
  setEventProcessingTime: () => {
    console.warn('setEventProcessingTime is not yet initialized.')
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
  const [pulseProcessingTime, setPulseProcessingTime] = useState<number | null>(null)
  const [eventProcessingTime, setEventProcessingTime] = useState<number | null>(null)
  const [decisionTrace, setDecisionTrace] = useState<DecisionTrace | null>(null)
  const [attemptTrace, setAttemptTrace] = useState<AttemptTrace | null>(null)

  useEffect(() => {
    /* Subscriber for pulse processing time. */
    const pulseProcessingTimeSubscriber = new Topic<Float64>({
      ros: ros,
      name: '/neurosimo/pipeline/processing_time/pulse',
      messageType: 'std_msgs/Float64',
    })

    pulseProcessingTimeSubscriber.subscribe((message) => {
      setPulseProcessingTime(message.data)
    })

    /* Subscriber for event processing time. */
    const eventProcessingTimeSubscriber = new Topic<Float64>({
      ros: ros,
      name: '/neurosimo/pipeline/processing_time/event',
      messageType: 'std_msgs/Float64',
    })

    eventProcessingTimeSubscriber.subscribe((message) => {
      setEventProcessingTime(message.data)
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
      pulseProcessingTimeSubscriber.unsubscribe()
      eventProcessingTimeSubscriber.unsubscribe()
      decisionTraceSubscriber.unsubscribe()
      attemptTraceSubscriber.unsubscribe()
    }
  }, [])

  return (
    <SessionStatisticsContext.Provider
      value={{
        pulseProcessingTime,
        eventProcessingTime,
        decisionTrace,
        attemptTrace,
        setPulseProcessingTime,
        setEventProcessingTime,
        setDecisionTrace,
        setAttemptTrace,
      }}
    >
      {children}
    </SessionStatisticsContext.Provider>
  )
}
