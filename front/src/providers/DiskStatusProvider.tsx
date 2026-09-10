import React, { useState, useEffect, ReactNode, useContext } from 'react'
import { Topic } from '@foxglove/roslibjs'

import { ros } from 'ros/ros'

export interface DiskStatus extends ROSLIB.Message {
  free_bytes: number
  total_bytes: number
  warning_threshold_bytes: number
  error_threshold_bytes: number
  is_ok: boolean
}

/* 'unknown' means that no disk status has been received yet; it is deliberately not treated
   as an error, so that a missing resource monitor does not block starting a session. */
export type DiskSeverity = 'unknown' | 'ok' | 'warning' | 'error'

export const getDiskSeverity = (diskStatus: DiskStatus | null): DiskSeverity => {
  if (!diskStatus) return 'unknown'
  if (diskStatus.free_bytes < diskStatus.error_threshold_bytes) return 'error'
  if (diskStatus.free_bytes < diskStatus.warning_threshold_bytes) return 'warning'
  return 'ok'
}

/* Format bytes as GiB, dropping a trailing '.0' so that, e.g., thresholds read as '50 GiB'. */
export const formatGiB = (bytes: number): string => {
  const gib = bytes / 1024 ** 3
  return gib.toFixed(1).replace(/\.0$/, '')
}

interface DiskStatusContextType {
  diskStatus: DiskStatus | null
  diskSeverity: DiskSeverity
}

const defaultDiskStatusState: DiskStatusContextType = {
  diskStatus: null,
  diskSeverity: 'unknown',
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
      name: '/neurosimo/system/disk_status',
      messageType: 'neurosimo_system_interfaces/DiskStatus',
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
        diskSeverity: getDiskSeverity(diskStatus),
      }}
    >
      {children}
    </DiskStatusContext.Provider>
  )
}

export const useDiskStatus = () => useContext(DiskStatusContext)