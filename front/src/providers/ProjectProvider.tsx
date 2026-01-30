import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from '@foxglove/roslibjs'

import { ros } from 'ros/ros'

interface RosString extends ROSLIB.Message {
  data: string
}

interface ProjectContextType {
  activeProject: string
}

const defaultProjectState: ProjectContextType = {
  activeProject: '',
}

export const ProjectContext = React.createContext<ProjectContextType>(defaultProjectState)

interface ProjectProviderProps {
  children: ReactNode
}

export const ProjectProvider: React.FC<ProjectProviderProps> = ({ children }) => {
  const [activeProject, setActiveProject] = useState<string>('')

  useEffect(() => {
    const activeProjectSubscriber = new Topic<RosString>({
      ros: ros,
      name: '/projects/active',
      messageType: 'std_msgs/String',
    })

    activeProjectSubscriber.subscribe((message) => {
      setActiveProject(message.data)
    })

    return () => {
      activeProjectSubscriber.unsubscribe()
    }
  }, [])

  return <ProjectContext.Provider value={{ activeProject }}>{children}</ProjectContext.Provider>
}
