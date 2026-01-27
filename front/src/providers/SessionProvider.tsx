import React, { useState, useEffect, ReactNode, createContext, useContext } from 'react'
import { startSessionRos, abortSessionRos, subscribeToSessionState, SessionState as SessionStateMessage } from 'ros/session'

export enum SessionStage {
  STOPPED = 0,
  INITIALIZING = 1,
  RUNNING = 2,
  FINALIZING = 3,
  ERROR = 4,
}

interface SessionState {
  isRunning: boolean
  stage: SessionStage
  message: string
}

interface SessionContextType {
  sessionState: SessionState
  startSession: (callback?: (success: boolean, message?: string) => void) => void
  abortSession: (callback?: (success: boolean, message?: string) => void) => void
}

// Default state
// eslint-disable-next-line @typescript-eslint/no-empty-function
const noopCallback = () => {}
const defaultSessionState: SessionContextType = {
  sessionState: {
    isRunning: false,
    stage: SessionStage.STOPPED,
    message: '',
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
    isRunning: false,
    stage: SessionStage.STOPPED,
    message: '',
  })

  useEffect(() => {
    // Subscribe to session state topic for persistence across refreshes
    const topic = subscribeToSessionState((state: SessionStateMessage) => {
      setSessionState({
        isRunning: state.is_running,
        stage: state.stage,
        message: state.message,
      })
    })

    return () => {
      topic.unsubscribe()
    }
  }, [])

  const startSession = (callback: (success: boolean, message?: string) => void = noopCallback) => {
    startSessionRos(callback)
  }

  const abortSession = (callback: (success: boolean, message?: string) => void = noopCallback) => {
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