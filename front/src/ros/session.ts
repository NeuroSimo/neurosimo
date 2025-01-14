import ROSLIB from 'roslib'
import { ros } from './ros'

export const startSessionService = new ROSLIB.Service({
  ros: ros,
  name: '/system/session/start',
  serviceType: 'system_interfaces/StartSession',
})

const stopSessionService = new ROSLIB.Service({
  ros: ros,
  name: '/system/session/stop',
  serviceType: 'system_interfaces/StopSession',
})

export const startSession = () => {
  const request = new ROSLIB.ServiceRequest({})
  startSessionService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to start session')
      }
    },
    (error) => {
      console.log('ERROR: Failed to start session')
      console.error(error)
    }
  )
}

export const stopSession = () => {
  const request = new ROSLIB.ServiceRequest({})
  stopSessionService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to stop session')
      }
    },
    (error) => {
      console.log('ERROR: Failed to stop session')
      console.error(error)
    }
  )
}
