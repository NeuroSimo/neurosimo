import React, { useState, useEffect, ReactNode, createContext, useContext } from 'react'
import { subscribeToExporterState, ExporterState } from 'ros/session'

export enum ExporterStateValue {
  IDLE = 0,
  EXPORTING = 1,
  ERROR = 2,
}

interface ExporterContextType {
  exporterState: ExporterState
}

const defaultExporterState: ExporterContextType = {
  exporterState: {
    state: ExporterStateValue.IDLE,
    recording_name: '',
    progress: 0,
  },
}

export const ExporterContext = createContext<ExporterContextType>(defaultExporterState)

interface ExporterProviderProps {
  children: ReactNode
}

export const ExporterProvider: React.FC<ExporterProviderProps> = ({ children }) => {
  const [exporterState, setExporterState] = useState<ExporterState>({
    state: ExporterStateValue.IDLE,
    recording_name: '',
    progress: 0,
  })

  useEffect(() => {
    const topic = subscribeToExporterState((state: ExporterState) => {
      setExporterState(state)
    })

    return () => {
      topic.unsubscribe()
    }
  }, [])

  const contextValue: ExporterContextType = {
    exporterState,
  }

  return (
    <ExporterContext.Provider value={contextValue}>
      {children}
    </ExporterContext.Provider>
  )
}

export const useExporter = () => {
  const context = useContext(ExporterContext)
  if (!context) {
    throw new Error('useExporter must be used within an ExporterProvider')
  }
  return context
}
