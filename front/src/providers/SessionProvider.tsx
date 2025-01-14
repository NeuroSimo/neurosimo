import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from 'roslib'

import { ros } from 'ros/ros'

/* Session */
export const SessionState = {
  STOPPED: 0,
  STARTING: 1,
  STARTED: 2,
  STOPPING: 3,
}

export const HumanReadableSessionState = {
  STOPPED: 'Stopped',
  STARTING: 'Starting...',
  STARTED: 'Started',
  STOPPING: 'Stopping...',
}

export interface Session extends Message {
  state: SessionState
  time: number
}

export interface SessionState {
  value: number
}

/* Context */
interface SessionContextType {
  session: Session | null
}

const defaultSessionState: SessionContextType = {
  session: null,
}

export const SessionContext = React.createContext<SessionContextType>(defaultSessionState)

interface SessionProviderProps {
  children: ReactNode
}

export const SessionProvider: React.FC<SessionProviderProps> = ({ children }) => {
  const [session, setSession] = useState<Session | null>(null)

  useEffect(() => {
    /* Subscriber for session. */
    const sessionSubscriber = new Topic<Session>({
      ros: ros,
      name: '/system/session',
      messageType: 'system_interfaces/Session',
    })

    sessionSubscriber.subscribe((message) => {
      setSession(message)
    })

    /* Unsubscribers */
    return () => {
      sessionSubscriber.unsubscribe()
    }
  }, [])

  return <SessionContext.Provider value={{ session }}>{children}</SessionContext.Provider>
}
