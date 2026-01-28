import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from 'roslib'

import { ros } from 'ros/ros'

export enum EegBridgeStateValue {
  READY = 0,
  LOADING = 1,
  RUNNING = 2,
  ERROR = 3,
}

interface RosDataSourceState extends ROSLIB.Message {
  state: EegBridgeStateValue
}

interface EegBridgeContextType {
  bridgeState: EegBridgeStateValue
}

const defaultBridgeState: EegBridgeContextType = {
  bridgeState: EegBridgeStateValue.READY,
}

export const EegBridgeContext = React.createContext<EegBridgeContextType>(defaultBridgeState)

interface EegBridgeProviderProps {
  children: ReactNode
}

export const EegBridgeProvider: React.FC<EegBridgeProviderProps> = ({ children }) => {
  const [bridgeState, setBridgeState] = useState<EegBridgeStateValue>(EegBridgeStateValue.READY)

  useEffect(() => {
    /* Subscriber for EEG bridge state. */
    const stateSubscriber = new Topic<RosDataSourceState>({
      ros: ros,
      name: '/eeg_bridge/state',
      messageType: 'system_interfaces/DataSourceState',
    })

    stateSubscriber.subscribe((message: RosDataSourceState) => {
      setBridgeState(message.state)
    })

    /* Unsubscriber */
    return () => {
      stateSubscriber.unsubscribe()
    }
  }, [])

  return (
    <EegBridgeContext.Provider
      value={{
        bridgeState,
      }}
    >
      {children}
    </EegBridgeContext.Provider>
  )
}