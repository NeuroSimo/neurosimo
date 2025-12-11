import ROSLIB from 'roslib'
import { ros } from './ros'

/* Start EEG bridge service */
const startEegBridgeService = new ROSLIB.Service({
  ros: ros,
  name: '/eeg_bridge/start',
  serviceType: 'std_srvs/Trigger',
})

export const startEegBridgeRos = (callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({}) as any

  startEegBridgeService.callService(
    request,
    (response: any) => {
      if (!response.success) {
        console.log('ERROR: Failed to start EEG bridge: success field was false.')
      } else {
        callback()
      }
    },
    (error: any) => {
      console.log('ERROR: Failed to start EEG bridge, error:')
      console.log(error)
    }
  )
}

/* Stop EEG bridge service */
const stopEegBridgeService = new ROSLIB.Service({
  ros: ros,
  name: '/eeg_bridge/stop',
  serviceType: 'std_srvs/Trigger',
})

export const stopEegBridgeRos = (callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({}) as any

  stopEegBridgeService.callService(
    request,
    (response: any) => {
      if (!response.success) {
        console.log('ERROR: Failed to stop EEG bridge: success field was false.')
      } else {
        callback()
      }
    },
    (error: any) => {
      console.log('ERROR: Failed to stop EEG bridge, error:')
      console.log(error)
    }
  )
}