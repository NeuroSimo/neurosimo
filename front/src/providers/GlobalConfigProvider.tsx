import React, { useState, useEffect, ReactNode, createContext, useContext } from 'react'
import { Topic } from '@foxglove/roslibjs'

import { ros } from 'ros/ros'

interface GlobalConfigContextType {
  activeProject: string
}

const defaultGlobalConfigState: GlobalConfigContextType = {
  activeProject: '',
}

export const GlobalConfigContext = createContext<GlobalConfigContextType>(defaultGlobalConfigState)

interface GlobalConfigProviderProps {
  children: ReactNode
}

export const GlobalConfigProvider: React.FC<GlobalConfigProviderProps> = ({ children }) => {
  const [globalConfig, setGlobalConfig] = useState<Map<string, string>>(new Map())

  const activeProject = globalConfig.get('active_project') || ''

  useEffect(() => {
    /* Subscriber for global config topic (latched). */
    const globalConfigSubscriber = new Topic({
      ros: ros,
      name: '/global_configurator/config',
      messageType: 'system_interfaces/GlobalConfig',
      queue_size: 1,
    })

    globalConfigSubscriber.subscribe((message: ROSLIB.Message) => {
      const msg = message as any
      setGlobalConfig((prevConfig) => {
        // Only update if active_project has changed
        if (prevConfig.get('active_project') === msg.active_project) {
          return prevConfig
        }
        
        const newConfig = new Map(prevConfig)
        newConfig.set('active_project', msg.active_project)
        return newConfig
      })
    })

    /* Cleanup */
    return () => {
      globalConfigSubscriber.unsubscribe()
    }
  }, [])

  return (
    <GlobalConfigContext.Provider
      value={{
        activeProject,
      }}
    >
      {children}
    </GlobalConfigContext.Provider>
  )
}

export const useGlobalConfig = () => {
  const context = useContext(GlobalConfigContext)
  if (!context) {
    throw new Error('useGlobalConfig must be used within a GlobalConfigProvider')
  }
  return context
}
