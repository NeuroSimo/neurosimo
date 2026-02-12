import React, { useState, useEffect, ReactNode } from 'react'
import { ros } from 'ros/ros'

interface RosConnectionContextType {
  isConnected: boolean
}

const RosConnectionContext = React.createContext<RosConnectionContextType>({
  isConnected: false
})

interface RosConnectionProviderProps {
  children: ReactNode
}

export const RosConnectionProvider: React.FC<RosConnectionProviderProps> = ({ children }) => {
  const [isConnected, setIsConnected] = useState(false)

  useEffect(() => {
    const handleConnection = () => setIsConnected(true)
    const handleClose = () => setIsConnected(false)
    const handleError = () => setIsConnected(false)

    // Check initial connection state
    setIsConnected(ros.isConnected || false)

    ros.on('connection', handleConnection)
    ros.on('close', handleClose)
    ros.on('error', handleError)

    return () => {
      ros.off('connection', handleConnection)
      ros.off('close', handleClose)
      ros.off('error', handleError)
    }
  }, [])

  return (
    <RosConnectionContext.Provider value={{ isConnected }}>
      {children}
    </RosConnectionContext.Provider>
  )
}

export const useRosConnection = () => {
  const context = React.useContext(RosConnectionContext)
  if (!context) {
    throw new Error('useRosConnection must be used within a RosConnectionProvider')
  }
  return context
}