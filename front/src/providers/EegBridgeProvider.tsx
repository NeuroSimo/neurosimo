import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from 'roslib'

import { ros } from 'ros/ros'
import { startEegBridgeRos, stopEegBridgeRos } from 'ros/eeg_bridge'

export enum EegBridgeStateValue {
  READY = 0,
  LOADING = 1,
  RUNNING = 2,
  ERROR = 3,
}

interface RosStreamerState extends ROSLIB.Message {
  state: EegBridgeStateValue
}

interface EegBridgeContextType {
  bridgeState: EegBridgeStateValue
  toggleStreaming: () => void
}

const defaultBridgeState: EegBridgeContextType = {
  bridgeState: EegBridgeStateValue.READY,
  toggleStreaming: () => {
    console.warn('toggleStreaming called before provider mounted')
  },
}

export const EegBridgeContext = React.createContext<EegBridgeContextType>(defaultBridgeState)

interface EegBridgeProviderProps {
  children: ReactNode
}

export const EegBridgeProvider: React.FC<EegBridgeProviderProps> = ({ children }) => {
  const [bridgeState, setBridgeState] = useState<EegBridgeStateValue>(EegBridgeStateValue.READY)

  useEffect(() => {
    /* Subscriber for EEG bridge state. */
    const stateSubscriber = new Topic<RosStreamerState>({
      ros: ros,
      name: '/eeg_bridge/state',
      messageType: 'system_interfaces/StreamerState',
    })

    stateSubscriber.subscribe((message: RosStreamerState) => {
      setBridgeState(message.state)
    })

    /* Unsubscriber */
    return () => {
      stateSubscriber.unsubscribe()
    }
  }, [])

  const toggleStreaming = () => {
    setBridgeState(EegBridgeStateValue.LOADING)

    if (bridgeState === EegBridgeStateValue.RUNNING) {
      // Service call to stop
      stopEegBridgeRos(() => {
        setBridgeState(EegBridgeStateValue.READY)
      })
    } else {
      // Service call to start
      startEegBridgeRos(() => {
        setBridgeState(EegBridgeStateValue.RUNNING)
      })
    }
  }

  return (
    <EegBridgeContext.Provider
      value={{
        bridgeState,
        toggleStreaming,
      }}
    >
      {children}
    </EegBridgeContext.Provider>
  )
}