import React, { useCallback, useEffect, useRef, useState, ReactNode } from 'react'
import ROSLIB from '@foxglove/roslibjs'

import { REQUIRED_NEUROSIMO_HEARTBEATS } from 'config/requiredHeartbeats'
import { ros } from 'ros/ros'
import { useRosConnection } from './RosConnectionProvider'

const STALE_MS = 1000
const TICK_MS = 200

interface HeartbeatContextType {
  waitingFor: string[]
}

const HeartbeatContext = React.createContext<HeartbeatContextType>({
  waitingFor: [],
})

interface Props {
  children: ReactNode
}

export const HeartbeatProvider: React.FC<Props> = ({ children }) => {
  const { isConnected } = useRosConnection()
  const [waitingFor, setWaitingFor] = useState<string[]>([])
  const gateStartRef = useRef<number | null>(null)
  const lastByIdRef = useRef<Partial<Record<string, number>>>({})

  const computeWaitingFor = useCallback((): string[] => {
    if (!isConnected) {
      return []
    }
    const gateStart = gateStartRef.current
    if (gateStart === null) {
      return []
    }
    const now = Date.now()
    return REQUIRED_NEUROSIMO_HEARTBEATS.filter(h => {
      const last = lastByIdRef.current[h.id]
      const anchor = last ?? gateStart
      return now - anchor > STALE_MS
    }).map(h => h.label)
  }, [isConnected])

  useEffect(() => {
    if (!isConnected) {
      gateStartRef.current = null
      lastByIdRef.current = {}
      setWaitingFor([])
      return
    }

    gateStartRef.current = Date.now()
    lastByIdRef.current = {}
    setWaitingFor([])

    const topics = REQUIRED_NEUROSIMO_HEARTBEATS.map(
      h =>
        new ROSLIB.Topic({
          ros,
          name: h.topic,
          messageType: 'std_msgs/Empty',
        }),
    )

    REQUIRED_NEUROSIMO_HEARTBEATS.forEach((h, i) => {
      topics[i].subscribe(() => {
        lastByIdRef.current[h.id] = Date.now()
        setWaitingFor(computeWaitingFor())
      })
    })

    const tick = setInterval(() => {
      setWaitingFor(computeWaitingFor())
    }, TICK_MS)

    return () => {
      clearInterval(tick)
      topics.forEach(t => t.unsubscribe())
    }
  }, [isConnected, computeWaitingFor])

  return (
    <HeartbeatContext.Provider value={{ waitingFor }}>{children}</HeartbeatContext.Provider>
  )
}

export const useHeartbeat = () => React.useContext(HeartbeatContext)
