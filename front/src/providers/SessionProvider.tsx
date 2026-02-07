import React, { useState, useEffect, ReactNode, createContext, useContext } from 'react'
import { startSessionRos, abortSessionRos, subscribeToSessionState } from 'ros/session'

export enum SessionStateValue {
  STOPPED = 0,
  INITIALIZING = 1,
  RUNNING = 2,
  FINALIZING = 3,
}

interface SessionState {
  state: SessionStateValue
  abortReason: string
}

interface SessionContextType {
  sessionState: SessionState
  startSession: (callback?: (success: boolean, message?: string) => void) => void
  abortSession: (callback?: (success: boolean) => void) => void
}

// Default state
// eslint-disable-next-line @typescript-eslint/no-empty-function
const noopCallback = () => {}
const defaultSessionState: SessionContextType = {
  sessionState: {
    state: SessionStateValue.STOPPED,
    abortReason: '',
  },
  startSession: noopCallback,
  abortSession: noopCallback,
}

export const SessionContext = createContext<SessionContextType>(defaultSessionState)

interface SessionProviderProps {
  children: ReactNode
}

export const SessionProvider: React.FC<SessionProviderProps> = ({ children }) => {
  const [sessionState, setSessionState] = useState<SessionState>({
    state: SessionStateValue.STOPPED,
    abortReason: '',
  })

  useEffect(() => {
    // Subscribe to session state topic for persistence across refreshes
    const topic = subscribeToSessionState((state: SessionStateValue, abortReason: string) => {
      setSessionState({
        state: state,
        abortReason: abortReason,
      })
    })

    return () => {
      topic.unsubscribe()
    }
  }, [])

  const startSession = (callback: (success: boolean, message?: string) => void = noopCallback) => {
    startSessionRos(callback)
  }

  const abortSession = (callback: (success: boolean) => void = noopCallback) => {
    abortSessionRos(callback)
  }

  const contextValue: SessionContextType = {
    sessionState,
    startSession,
    abortSession,
  }

  return (
    <SessionContext.Provider value={contextValue}>
      {children}
    </SessionContext.Provider>
  )
}

export const useSession = () => {
  const context = useContext(SessionContext)
  if (!context) {
    throw new Error('useSession must be used within a SessionProvider')
  }
  return context
}