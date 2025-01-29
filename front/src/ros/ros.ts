import React from 'react'
import ROSLIB from 'roslib'

const ROSBRIDGE_URL = `${process.env.REACT_APP_ROSBRIDGE_URL}`

if (ROSBRIDGE_URL === 'undefined') {
  throw new Error('ROSBRIDGE_URL environment variable is not set')
}

console.log('Connecting to ROS at', ROSBRIDGE_URL)

export const ros = new ROSLIB.Ros({
  url: ROSBRIDGE_URL,
})

ros.on('connection', () => {
  console.log('Connected to ROS', ROSBRIDGE_URL)
})

ros.on('error', (error) => {
  console.log('Error connecting to ROS, error:', error)
})

ros.on('close', () => {
  console.log('Connection to ROS closed')
})
