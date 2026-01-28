import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from 'roslib'

import { ros } from 'ros/ros'

export const ComponentHealth = {
  READY: 'ready',
  DEGRADED: 'degraded',
  ERROR: 'error',
  UNKNOWN: 'unknown',
} as const

type ComponentHealthType = typeof ComponentHealth[keyof typeof ComponentHealth]

interface HeartbeatMessage extends ROSLIB.Message {}

interface ComponentHealthMessage extends ROSLIB.Message {
  health_level: number
  message: string
}

interface ComponentHealth {
  health: ComponentHealthType
  message: string
  lastHeartbeat: number | null
}

interface HealthcheckContextType {
  eegBridgeStatus: ComponentHealth
  eegSimulatorStatus: ComponentHealth
  preprocessorStatus: ComponentHealth
  deciderStatus: ComponentHealth
  experimentCoordinatorStatus: ComponentHealth
  resourceMonitorStatus: ComponentHealth
  presenterStatus: ComponentHealth
  triggerTimerStatus: ComponentHealth
}

const HEARTBEAT_TIMEOUT_MS = 800

const defaultComponentHealth: ComponentHealth = {
  health: ComponentHealth.UNKNOWN,
  message: '',
  lastHeartbeat: null,
}

const defaultHealthcheckState: HealthcheckContextType = {
  eegBridgeStatus: { ...defaultComponentHealth },
  eegSimulatorStatus: { ...defaultComponentHealth },
  preprocessorStatus: { ...defaultComponentHealth },
  deciderStatus: { ...defaultComponentHealth },
  experimentCoordinatorStatus: { ...defaultComponentHealth },
  resourceMonitorStatus: { ...defaultComponentHealth },
  presenterStatus: { ...defaultComponentHealth },
  triggerTimerStatus: { ...defaultComponentHealth },
}

export const HealthcheckContext = React.createContext<HealthcheckContextType>(defaultHealthcheckState)

interface HealthcheckProviderProps {
  children: ReactNode
}

export const HealthcheckProvider: React.FC<HealthcheckProviderProps> = ({ children }) => {
  const [eegBridgeStatus, setEegBridgeStatus] = useState<ComponentHealth>({ ...defaultComponentHealth })
  const [eegSimulatorStatus, setEegSimulatorStatus] = useState<ComponentHealth>({ ...defaultComponentHealth })
  const [preprocessorStatus, setPreprocessorStatus] = useState<ComponentHealth>({ ...defaultComponentHealth })
  const [deciderStatus, setDeciderStatus] = useState<ComponentHealth>({ ...defaultComponentHealth })
  const [experimentCoordinatorStatus, setExperimentCoordinatorStatus] = useState<ComponentHealth>({ ...defaultComponentHealth })
  const [resourceMonitorStatus, setResourceMonitorStatus] = useState<ComponentHealth>({ ...defaultComponentHealth })
  const [presenterStatus, setPresenterStatus] = useState<ComponentHealth>({ ...defaultComponentHealth })
  const [triggerTimerStatus, setTriggerTimerStatus] = useState<ComponentHealth>({ ...defaultComponentHealth })

  const updateHeartbeatStatus = (setter: React.Dispatch<React.SetStateAction<ComponentHealth>>) => {
    const now = Date.now()
    setter((prev) => ({
      ...prev,
      lastHeartbeat: now,
    }))
  }

  const updateHealthStatus = (setter: React.Dispatch<React.SetStateAction<ComponentHealth>>, message: ComponentHealthMessage) => {
    let health: ComponentHealthType
    switch (message.health_level) {
      case 0: // READY
        health = ComponentHealth.READY
        break
      case 1: // DEGRADED
        health = ComponentHealth.DEGRADED
        break
      case 2: // ERROR
        health = ComponentHealth.ERROR
        break
      default:
        health = ComponentHealth.UNKNOWN
        break
    }
    setter((prev) => ({
      ...prev,
      health,
      message: message.message,
    }))
  }

  const checkTimeouts = () => {
    const now = Date.now()

    const updateHealthStatus = (status: ComponentHealth, setter: React.Dispatch<React.SetStateAction<ComponentHealth>>) => {
      if (status.lastHeartbeat && now - status.lastHeartbeat > HEARTBEAT_TIMEOUT_MS) {
        setter({
          ...status,
          health: ComponentHealth.UNKNOWN,
        })
      }
    }

    updateHealthStatus(eegBridgeStatus, setEegBridgeStatus)
    updateHealthStatus(eegSimulatorStatus, setEegSimulatorStatus)
    updateHealthStatus(preprocessorStatus, setPreprocessorStatus)
    updateHealthStatus(deciderStatus, setDeciderStatus)
    updateHealthStatus(experimentCoordinatorStatus, setExperimentCoordinatorStatus)
    updateHealthStatus(resourceMonitorStatus, setResourceMonitorStatus)
    updateHealthStatus(presenterStatus, setPresenterStatus)
    updateHealthStatus(triggerTimerStatus, setTriggerTimerStatus)
  }

  useEffect(() => {
    // Heartbeat subscribers
    const eegBridgeHeartbeatSubscriber = new Topic<HeartbeatMessage>({
      ros: ros,
      name: '/eeg_bridge/heartbeat',
      messageType: 'std_msgs/Empty',
    })

    const eegSimulatorHeartbeatSubscriber = new Topic<HeartbeatMessage>({
      ros: ros,
      name: '/eeg_simulator/heartbeat',
      messageType: 'std_msgs/Empty',
    })

    const preprocessorHeartbeatSubscriber = new Topic<HeartbeatMessage>({
      ros: ros,
      name: '/essor/heartbeat',
      messageType: 'std_msgs/Empty',
    })

    const deciderHeartbeatSubscriber = new Topic<HeartbeatMessage>({
      ros: ros,
      name: '/decider/heartbeat',
      messageType: 'std_msgs/Empty',
    })

    const experimentCoordinatorHeartbeatSubscriber = new Topic<HeartbeatMessage>({
      ros: ros,
      name: '/experiment_coordinator/heartbeat',
      messageType: 'std_msgs/Empty',
    })

    const resourceMonitorHeartbeatSubscriber = new Topic<HeartbeatMessage>({
      ros: ros,
      name: '/resource_monitor/heartbeat',
      messageType: 'std_msgs/Empty',
    })

    const presenterHeartbeatSubscriber = new Topic<HeartbeatMessage>({
      ros: ros,
      name: '/presenter/heartbeat',
      messageType: 'std_msgs/Empty',
    })

    const triggerTimerHeartbeatSubscriber = new Topic<HeartbeatMessage>({
      ros: ros,
      name: '/trigger_timer/heartbeat',
      messageType: 'std_msgs/Empty',
    })

    // Health status subscribers
    const eegBridgeHealthSubscriber = new Topic<ComponentHealthMessage>({
      ros: ros,
      name: '/eeg_bridge/health',
      messageType: 'system_interfaces/ComponentHealth',
    })

    const eegSimulatorHealthSubscriber = new Topic<ComponentHealthMessage>({
      ros: ros,
      name: '/eeg_simulator/health',
      messageType: 'system_interfaces/ComponentHealth',
    })

    const preprocessorHealthSubscriber = new Topic<ComponentHealthMessage>({
      ros: ros,
      name: '/preprocessor/health',
      messageType: 'system_interfaces/ComponentHealth',
    })

    const deciderHealthSubscriber = new Topic<ComponentHealthMessage>({
      ros: ros,
      name: '/decider/health',
      messageType: 'system_interfaces/ComponentHealth',
    })

    const experimentCoordinatorHealthSubscriber = new Topic<ComponentHealthMessage>({
      ros: ros,
      name: '/experiment_coordinator/health',
      messageType: 'system_interfaces/ComponentHealth',
    })

    const resourceMonitorHealthSubscriber = new Topic<ComponentHealthMessage>({
      ros: ros,
      name: '/resource_monitor/health',
      messageType: 'system_interfaces/ComponentHealth',
    })

    const presenterHealthSubscriber = new Topic<ComponentHealthMessage>({
      ros: ros,
      name: '/presenter/health',
      messageType: 'system_interfaces/ComponentHealth',
    })

    const triggerTimerHealthSubscriber = new Topic<ComponentHealthMessage>({
      ros: ros,
      name: '/trigger_timer/health',
      messageType: 'system_interfaces/ComponentHealth',
    })

    // Subscribe to heartbeats
    eegBridgeHeartbeatSubscriber.subscribe(() => {
      updateHeartbeatStatus(setEegBridgeStatus)
    })

    eegSimulatorHeartbeatSubscriber.subscribe(() => {
      updateHeartbeatStatus(setEegSimulatorStatus)
    })

    preprocessorHeartbeatSubscriber.subscribe(() => {
      updateHeartbeatStatus(setPreprocessorStatus)
    })

    deciderHeartbeatSubscriber.subscribe(() => {
      updateHeartbeatStatus(setDeciderStatus)
    })

    experimentCoordinatorHeartbeatSubscriber.subscribe(() => {
      updateHeartbeatStatus(setExperimentCoordinatorStatus)
    })

    resourceMonitorHeartbeatSubscriber.subscribe(() => {
      updateHeartbeatStatus(setResourceMonitorStatus)
    })

    presenterHeartbeatSubscriber.subscribe(() => {
      updateHeartbeatStatus(setPresenterStatus)
    })

    triggerTimerHeartbeatSubscriber.subscribe(() => {
      updateHeartbeatStatus(setTriggerTimerStatus)
    })

    // Subscribe to health status
    eegBridgeHealthSubscriber.subscribe((message: ComponentHealthMessage) => {
      updateHealthStatus(setEegBridgeStatus, message)
    })

    eegSimulatorHealthSubscriber.subscribe((message: ComponentHealthMessage) => {
      updateHealthStatus(setEegSimulatorStatus, message)
    })

    preprocessorHealthSubscriber.subscribe((message: ComponentHealthMessage) => {
      updateHealthStatus(setPreprocessorStatus, message)
    })

    deciderHealthSubscriber.subscribe((message: ComponentHealthMessage) => {
      updateHealthStatus(setDeciderStatus, message)
    })

    experimentCoordinatorHealthSubscriber.subscribe((message: ComponentHealthMessage) => {
      updateHealthStatus(setExperimentCoordinatorStatus, message)
    })

    resourceMonitorHealthSubscriber.subscribe((message: ComponentHealthMessage) => {
      updateHealthStatus(setResourceMonitorStatus, message)
    })

    presenterHealthSubscriber.subscribe((message: ComponentHealthMessage) => {
      updateHealthStatus(setPresenterStatus, message)
    })

    triggerTimerHealthSubscriber.subscribe((message: ComponentHealthMessage) => {
      updateHealthStatus(setTriggerTimerStatus, message)
    })

    // Check for timeouts every 100ms
    const timeoutChecker = setInterval(checkTimeouts, 100)

    return () => {
      eegBridgeHeartbeatSubscriber.unsubscribe()
      eegSimulatorHeartbeatSubscriber.unsubscribe()
      preprocessorHeartbeatSubscriber.unsubscribe()
      deciderHeartbeatSubscriber.unsubscribe()
      experimentCoordinatorHeartbeatSubscriber.unsubscribe()
      resourceMonitorHeartbeatSubscriber.unsubscribe()
      presenterHeartbeatSubscriber.unsubscribe()
      triggerTimerHeartbeatSubscriber.unsubscribe()
      eegBridgeHealthSubscriber.unsubscribe()
      eegSimulatorHealthSubscriber.unsubscribe()
      preprocessorHealthSubscriber.unsubscribe()
      deciderHealthSubscriber.unsubscribe()
      experimentCoordinatorHealthSubscriber.unsubscribe()
      resourceMonitorHealthSubscriber.unsubscribe()
      presenterHealthSubscriber.unsubscribe()
      triggerTimerHealthSubscriber.unsubscribe()
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
        presenterStatus,
        triggerTimerStatus,
      }}
    >
      {children}
    </HealthcheckContext.Provider>
  )
}
