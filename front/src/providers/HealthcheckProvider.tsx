import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from 'roslib'

import { ros } from 'ros/ros'

export const HealthcheckStatus = {
  READY: 0,
  NOT_READY: 1,
  DISABLED: 2,
  ERROR: 3,
}

interface HealthcheckStatusMessage {
  value: number
}

interface Healthcheck extends ROSLIB.Message {
  status: HealthcheckStatusMessage
  status_message: string
  actionable_message: string
}

interface HealthcheckContextType {
  eegHealthcheck: Healthcheck | null
  eegSimulatorHealthcheck: Healthcheck | null
  preprocessorHealthcheck: Healthcheck | null
  deciderHealthcheck: Healthcheck | null
  resourceMonitorHealthcheck: Healthcheck | null
}

const defaultHealthcheckState: HealthcheckContextType = {
  eegHealthcheck: null,
  eegSimulatorHealthcheck: null,
  preprocessorHealthcheck: null,
  deciderHealthcheck: null,
  resourceMonitorHealthcheck: null,
}

export const HealthcheckContext = React.createContext<HealthcheckContextType>(defaultHealthcheckState)

interface HealthcheckProviderProps {
  children: ReactNode
}

export const HealthcheckProvider: React.FC<HealthcheckProviderProps> = ({ children }) => {
  const [eegHealthcheck, setEegHealthcheck] = useState<Healthcheck | null>(null)
  const [eegSimulatorHealthcheck, setEegSimulatorHealthcheck] = useState<Healthcheck | null>(null)
  const [preprocessorHealthcheck, setPreprocessorHealthcheck] = useState<Healthcheck | null>(null)
  const [deciderHealthcheck, setDeciderHealthcheck] = useState<Healthcheck | null>(null)
  const [resourceMonitorHealthcheck, setResourceMonitorHealthcheck] = useState<Healthcheck | null>(null)

  useEffect(() => {
    const eegSubscriber = new Topic<Healthcheck>({
      ros: ros,
      name: '/eeg/healthcheck',
      messageType: 'system_interfaces/Healthcheck',
    })

    const eegSimulatorSubscriber = new Topic<Healthcheck>({
      ros: ros,
      name: '/eeg_simulator/healthcheck',
      messageType: 'system_interfaces/Healthcheck',
    })

    const preprocessorSubscriber = new Topic<Healthcheck>({
      ros: ros,
      name: '/eeg/preprocessor/healthcheck',
      messageType: 'system_interfaces/Healthcheck',
    })

    const deciderSubscriber = new Topic<Healthcheck>({
      ros: ros,
      name: '/eeg/decider/healthcheck',
      messageType: 'system_interfaces/Healthcheck',
    })

    const resourceMonitorSubscriber = new Topic<Healthcheck>({
      ros: ros,
      name: '/system/resource_monitor/healthcheck',
      messageType: 'system_interfaces/Healthcheck',
    })

    eegSubscriber.subscribe((message) => {
      setEegHealthcheck(message)
    })

    eegSimulatorSubscriber.subscribe((message) => {
      setEegSimulatorHealthcheck(message)
    })

    preprocessorSubscriber.subscribe((message) => {
      setPreprocessorHealthcheck(message)
    })

    deciderSubscriber.subscribe((message) => {
      setDeciderHealthcheck(message)
    })

    resourceMonitorSubscriber.subscribe((message) => {
      setResourceMonitorHealthcheck(message)
    })

    return () => {
      eegSubscriber.unsubscribe()
      eegSimulatorSubscriber.unsubscribe()
      preprocessorSubscriber.unsubscribe()
      deciderSubscriber.unsubscribe()
      resourceMonitorSubscriber.unsubscribe()
    }
  }, [])

  return (
    <HealthcheckContext.Provider
      value={{
        eegHealthcheck,
        eegSimulatorHealthcheck,
        preprocessorHealthcheck,
        deciderHealthcheck,
        resourceMonitorHealthcheck,
      }}
    >
      {children}
    </HealthcheckContext.Provider>
  )
}
