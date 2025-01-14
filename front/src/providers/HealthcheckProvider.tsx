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
  mtmsDeviceHealthcheck: Healthcheck | null
  mepHealthcheck: Healthcheck | null
  preprocessorHealthcheck: Healthcheck | null
  deciderHealthcheck: Healthcheck | null
}

const defaultHealthcheckState: HealthcheckContextType = {
  eegHealthcheck: null,
  eegSimulatorHealthcheck: null,
  mtmsDeviceHealthcheck: null,
  mepHealthcheck: null,
  preprocessorHealthcheck: null,
  deciderHealthcheck: null,
}

export const HealthcheckContext = React.createContext<HealthcheckContextType>(defaultHealthcheckState)

interface HealthcheckProviderProps {
  children: ReactNode
}

export const HealthcheckProvider: React.FC<HealthcheckProviderProps> = ({ children }) => {
  const [eegHealthcheck, setEegHealthcheck] = useState<Healthcheck | null>(null)
  const [eegSimulatorHealthcheck, setEegSimulatorHealthcheck] = useState<Healthcheck | null>(null)
  const [mtmsDeviceHealthcheck, setMtmsDeviceHealthcheck] = useState<Healthcheck | null>(null)
  const [mepHealthcheck, setMepHealthcheck] = useState<Healthcheck | null>(null)
  const [preprocessorHealthcheck, setPreprocessorHealthcheck] = useState<Healthcheck | null>(null)
  const [deciderHealthcheck, setDeciderHealthcheck] = useState<Healthcheck | null>(null)

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

    const mtmsSubscriber = new Topic<Healthcheck>({
      ros: ros,
      name: '/mtms_device/healthcheck',
      messageType: 'system_interfaces/Healthcheck',
    })

    const mepSubscriber = new Topic<Healthcheck>({
      ros: ros,
      name: '/mep/healthcheck',
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

    let eegTimeout: NodeJS.Timeout | null = null
    let eegSimulatorTimeout: NodeJS.Timeout | null = null
    let mtmsTimeout: NodeJS.Timeout | null = null
    let mepTimeout: NodeJS.Timeout | null = null
    let preprocessorTimeout: NodeJS.Timeout | null = null
    let deciderTimeout: NodeJS.Timeout | null = null

    eegSubscriber.subscribe((message) => {
      setEegHealthcheck(message)
      if (eegTimeout) {
        clearTimeout(eegTimeout)
      }
      eegTimeout = setTimeout(() => {
        setEegHealthcheck(null)
      }, 1200)
    })

    eegSimulatorSubscriber.subscribe((message) => {
      setEegSimulatorHealthcheck(message)
      if (eegSimulatorTimeout) {
        clearTimeout(eegSimulatorTimeout)
      }
      eegSimulatorTimeout = setTimeout(() => {
        setEegSimulatorHealthcheck(null)
      }, 1200)
    })

    mtmsSubscriber.subscribe((message) => {
      setMtmsDeviceHealthcheck(message)
      if (mtmsTimeout) {
        clearTimeout(mtmsTimeout)
      }
      mtmsTimeout = setTimeout(() => {
        setMtmsDeviceHealthcheck(null)
      }, 1200)
    })

    mepSubscriber.subscribe((message) => {
      setMepHealthcheck(message)
      if (mepTimeout) {
        clearTimeout(mepTimeout)
      }
      mepTimeout = setTimeout(() => {
        setMepHealthcheck(null)
      }, 1200)
    })

    preprocessorSubscriber.subscribe((message) => {
      setPreprocessorHealthcheck(message)
      if (preprocessorTimeout) {
        clearTimeout(preprocessorTimeout)
      }
      preprocessorTimeout = setTimeout(() => {
        setPreprocessorHealthcheck(null)
      }, 1200)
    })

    deciderSubscriber.subscribe((message) => {
      setDeciderHealthcheck(message)
      if (deciderTimeout) {
        clearTimeout(deciderTimeout)
      }
      deciderTimeout = setTimeout(() => {
        setDeciderHealthcheck(null)
      }, 1200)
    })

    return () => {
      eegSubscriber.unsubscribe()
      eegSimulatorSubscriber.unsubscribe()
      mtmsSubscriber.unsubscribe()
      mepSubscriber.unsubscribe()
      preprocessorSubscriber.unsubscribe()
      deciderSubscriber.unsubscribe()
      if (eegTimeout) {
        clearTimeout(eegTimeout)
      }
      if (eegSimulatorTimeout) {
        clearTimeout(eegSimulatorTimeout)
      }
      if (mtmsTimeout) {
        clearTimeout(mtmsTimeout)
      }
      if (mepTimeout) {
        clearTimeout(mepTimeout)
      }
      if (preprocessorTimeout) {
        clearTimeout(preprocessorTimeout)
      }
      if (deciderTimeout) {
        clearTimeout(deciderTimeout)
      }
    }
  }, [])

  return (
    <HealthcheckContext.Provider
      value={{
        eegHealthcheck,
        eegSimulatorHealthcheck,
        mtmsDeviceHealthcheck,
        mepHealthcheck,
        preprocessorHealthcheck,
        deciderHealthcheck,
      }}
    >
      {children}
    </HealthcheckContext.Provider>
  )
}
