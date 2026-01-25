import React, { useState, useEffect, ReactNode } from 'react'
import { Topic } from 'roslib'

import { ros } from 'ros/ros'

interface DiskStatus extends ROSLIB.Message {
  free_bytes: number
  total_bytes: number
  warning_threshold_bytes: number
  error_threshold_bytes: number
  is_ok: boolean
}

interface DiskStatusContextType {
  diskStatus: DiskStatus | null
}

const defaultDiskStatusState: DiskStatusContextType = {
  diskStatus: null,
}

export const DiskStatusContext = React.createContext<DiskStatusContextType>(defaultDiskStatusState)

interface DiskStatusProviderProps {
  children: ReactNode
}

export const DiskStatusProvider: React.FC<DiskStatusProviderProps> = ({ children }) => {
  const [diskStatus, setDiskStatus] = useState<DiskStatus | null>(null)

  useEffect(() => {
    const diskStatusSubscriber = new Topic<DiskStatus>({
      ros: ros,
      name: '/system/disk_status',
      messageType: 'system_interfaces/DiskStatus',
    })

    diskStatusSubscriber.subscribe((message) => {
      setDiskStatus(message)
    })

    return () => {
      diskStatusSubscriber.unsubscribe()
    }
  }, [])

  return (
    <DiskStatusContext.Provider
      value={{
        diskStatus,
      }}
    >
      {children}
    </DiskStatusContext.Provider>
  )
}