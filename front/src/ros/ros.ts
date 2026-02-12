import React from 'react'
import ROSLIB from '@foxglove/roslibjs'

const ROSBRIDGE_URL = 'ws://localhost:9090'

if (!ROSBRIDGE_URL) {
  throw new Error('ROSBRIDGE_URL environment variable is not set')
}

console.log('Connecting to ROS at', ROSBRIDGE_URL)

export const ros = new ROSLIB.Ros({
  url: ROSBRIDGE_URL,
})

// Auto-reconnection logic
let reconnectTimeout: NodeJS.Timeout | null = null
const RECONNECT_INTERVAL = 5000 // 5 seconds between attempts
const MAX_RECONNECT_ATTEMPTS = 30 // Give up after 30 attempts (~2.5 minutes)

let reconnectAttempts = 0
let isReconnecting = false

function scheduleReconnect() {
  if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS || isReconnecting) {
    if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS) {
      console.error('Failed to reconnect to ROS after', MAX_RECONNECT_ATTEMPTS, 'attempts')
    }
    return
  }

  isReconnecting = true
  reconnectAttempts++
  console.log(`Reconnecting to ROS (${reconnectAttempts}/${MAX_RECONNECT_ATTEMPTS})...`)

  reconnectTimeout = setTimeout(() => {
    isReconnecting = false
    ros.connect(ROSBRIDGE_URL)
  }, RECONNECT_INTERVAL)
}

ros.on('connection', () => {
  console.log('Connected to ROS', ROSBRIDGE_URL)
  reconnectAttempts = 0
  isReconnecting = false
  if (reconnectTimeout) {
    clearTimeout(reconnectTimeout)
    reconnectTimeout = null
  }
})

ros.on('error', (error) => {
  console.log('ROS connection error:', error)
  if (!isReconnecting) {
    scheduleReconnect()
  }
})

ros.on('close', () => {
  console.log('ROS connection closed - will reconnect...')
  if (!isReconnecting) {
    scheduleReconnect()
  }
})
