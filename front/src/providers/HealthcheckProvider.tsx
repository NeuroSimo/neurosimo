import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from 'roslib'

import { ros } from 'ros/ros'

export const ComponentHealth = {
  HEALTHY: 'healthy',
  UNHEALTHY: 'unhealthy',
  UNKNOWN: 'unknown',
} as const

type ComponentHealthType = typeof ComponentHealth[keyof typeof ComponentHealth]

interface HeartbeatMessage extends ROSLIB.Message {}

interface ComponentStatus {
  health: ComponentHealthType
  lastHeartbeat: number | null
}

interface HealthcheckContextType {
  eegBridgeStatus: ComponentStatus
  eegSimulatorStatus: ComponentStatus
  preprocessorStatus: ComponentStatus
  deciderStatus: ComponentStatus
  experimentCoordinatorStatus: ComponentStatus
  resourceMonitorStatus: ComponentStatus
}

const HEARTBEAT_TIMEOUT_MS = 800

const defaultComponentStatus: ComponentStatus = {
  health: ComponentHealth.UNKNOWN,
  lastHeartbeat: null,
}

const defaultHealthcheckState: HealthcheckContextType = {
  eegBridgeStatus: { ...defaultComponentStatus },
  eegSimulatorStatus: { ...defaultComponentStatus },
  preprocessorStatus: { ...defaultComponentStatus },
  deciderStatus: { ...defaultComponentStatus },
  experimentCoordinatorStatus: { ...defaultComponentStatus },
  resourceMonitorStatus: { ...defaultComponentStatus },
}

export const HealthcheckContext = React.createContext<HealthcheckContextType>(defaultHealthcheckState)

interface HealthcheckProviderProps {
  children: ReactNode
}

export const HealthcheckProvider: React.FC<HealthcheckProviderProps> = ({ children }) => {
  const [eegBridgeStatus, setEegBridgeStatus] = useState<ComponentStatus>({ ...defaultComponentStatus })
  const [eegSimulatorStatus, setEegSimulatorStatus] = useState<ComponentStatus>({ ...defaultComponentStatus })
  const [preprocessorStatus, setPreprocessorStatus] = useState<ComponentStatus>({ ...defaultComponentStatus })
  const [deciderStatus, setDeciderStatus] = useState<ComponentStatus>({ ...defaultComponentStatus })
  const [experimentCoordinatorStatus, setExperimentCoordinatorStatus] = useState<ComponentStatus>({ ...defaultComponentStatus })
  const [resourceMonitorStatus, setResourceMonitorStatus] = useState<ComponentStatus>({ ...defaultComponentStatus })

  const updateComponentStatus = (setter: React.Dispatch<React.SetStateAction<ComponentStatus>>) => {
    const now = Date.now()
    setter({
      health: ComponentHealth.HEALTHY,
      lastHeartbeat: now,
    })
  }

  const checkTimeouts = () => {
    const now = Date.now()

    const updateIfTimedOut = (status: ComponentStatus, setter: React.Dispatch<React.SetStateAction<ComponentStatus>>) => {
      if (status.lastHeartbeat && now - status.lastHeartbeat > HEARTBEAT_TIMEOUT_MS) {
        setter({
          ...status,
          health: ComponentHealth.UNHEALTHY,
        })
      }
    }

    updateIfTimedOut(eegBridgeStatus, setEegBridgeStatus)
    updateIfTimedOut(eegSimulatorStatus, setEegSimulatorStatus)
    updateIfTimedOut(preprocessorStatus, setPreprocessorStatus)
    updateIfTimedOut(deciderStatus, setDeciderStatus)
    updateIfTimedOut(experimentCoordinatorStatus, setExperimentCoordinatorStatus)
    updateIfTimedOut(resourceMonitorStatus, setResourceMonitorStatus)
  }

  useEffect(() => {
    const eegBridgeSubscriber = new Topic<HeartbeatMessage>({
      ros: ros,
      name: '/health/eeg_bridge/heartbeat',
      messageType: 'std_msgs/Empty',
    })

    const eegSimulatorSubscriber = new Topic<HeartbeatMessage>({
      ros: ros,
      name: '/health/eeg_simulator/heartbeat',
      messageType: 'std_msgs/Empty',
    })

    const preprocessorSubscriber = new Topic<HeartbeatMessage>({
      ros: ros,
      name: '/health/preprocessor/heartbeat',
      messageType: 'std_msgs/Empty',
    })

    const deciderSubscriber = new Topic<HeartbeatMessage>({
      ros: ros,
      name: '/health/decider/heartbeat',
      messageType: 'std_msgs/Empty',
    })

    const experimentCoordinatorSubscriber = new Topic<HeartbeatMessage>({
      ros: ros,
      name: '/health/experiment_coordinator/heartbeat',
      messageType: 'std_msgs/Empty',
    })

    const resourceMonitorSubscriber = new Topic<HeartbeatMessage>({
      ros: ros,
      name: '/health/resource_monitor/heartbeat',
      messageType: 'std_msgs/Empty',
    })

    eegBridgeSubscriber.subscribe(() => {
      updateComponentStatus(setEegBridgeStatus)
    })

    eegSimulatorSubscriber.subscribe(() => {
      updateComponentStatus(setEegSimulatorStatus)
    })

    preprocessorSubscriber.subscribe(() => {
      updateComponentStatus(setPreprocessorStatus)
    })

    deciderSubscriber.subscribe(() => {
      updateComponentStatus(setDeciderStatus)
    })

    experimentCoordinatorSubscriber.subscribe(() => {
      updateComponentStatus(setExperimentCoordinatorStatus)
    })

    resourceMonitorSubscriber.subscribe(() => {
      updateComponentStatus(setResourceMonitorStatus)
    })

    // Check for timeouts every 100ms
    const timeoutChecker = setInterval(checkTimeouts, 100)

    return () => {
      eegBridgeSubscriber.unsubscribe()
      eegSimulatorSubscriber.unsubscribe()
      preprocessorSubscriber.unsubscribe()
      deciderSubscriber.unsubscribe()
      experimentCoordinatorSubscriber.unsubscribe()
      resourceMonitorSubscriber.unsubscribe()
      clearInterval(timeoutChecker)
    }
  }, [])

  return (
    <HealthcheckContext.Provider
      value={{
        eegBridgeStatus,
        eegSimulatorStatus,
        preprocessorStatus,
        deciderStatus,
        experimentCoordinatorStatus,
        resourceMonitorStatus,
      }}
    >
      {children}
    </HealthcheckContext.Provider>
  )
}
