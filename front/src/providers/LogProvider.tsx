import React, { useState, useEffect, ReactNode } from 'react'
import { Topic } from '@foxglove/roslibjs'

import { ros } from 'ros/ros'

export interface LogMessage extends ROSLIB.Message {
  message: string
  sample_time: number
  level: number  // 0 = INFO, 1 = WARNING, 2 = ERROR
  phase: number  // 0 = RUNTIME, 1 = INITIALIZATION, 2 = FINALIZATION
}

export interface LogMessages extends ROSLIB.Message {
  messages: LogMessage[]
}

export const LogLevel = {
  INFO: 0,
  WARNING: 1,
  ERROR: 2,
} as const

export const LogPhase = {
  RUNTIME: 0,
  INITIALIZATION: 1,
  FINALIZATION: 2,
} as const

interface LogContextType {
  preprocessorLogs: LogMessage[]
  deciderLogs: LogMessage[]
  presenterLogs: LogMessage[]
  clearAllLogs: () => void
}

const defaultLogState: LogContextType = {
  preprocessorLogs: [],
  deciderLogs: [],
  presenterLogs: [],
  clearAllLogs: () => {
    console.warn('clearAllLogs is not yet initialized.')
  },
}

export const LogContext = React.createContext<LogContextType>(defaultLogState)

interface LogProviderProps {
  children: ReactNode
}

export const LogProvider: React.FC<LogProviderProps> = ({ children }) => {
  const [preprocessorLogs, setPreprocessorLogs] = useState<LogMessage[]>([])
  const [deciderLogs, setDeciderLogs] = useState<LogMessage[]>([])
  const [presenterLogs, setPresenterLogs] = useState<LogMessage[]>([])

  const clearAllLogs = () => {
    setPreprocessorLogs([])
    setDeciderLogs([])
    setPresenterLogs([])
  }

  useEffect(() => {
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

    /* Unsubscribers */
    return () => {
      preprocessorLogSubscriber.unsubscribe()
      deciderLogSubscriber.unsubscribe()
      presenterLogSubscriber.unsubscribe()
    }
  }, [])

  return (
    <LogContext.Provider
      value={{
        preprocessorLogs,
        deciderLogs,
        presenterLogs,
        clearAllLogs,
      }}
    >
      {children}
    </LogContext.Provider>
  )
}