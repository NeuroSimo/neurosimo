import ROSLIB from 'roslib'
import { ros } from './ros'

/* Pause experiment */
const pauseExperimentService = new ROSLIB.Service({
  ros: ros,
  name: '/experiment/pause',
  serviceType: 'std_srvs/Trigger',
})

export const pauseExperimentRos = (callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({}) as any

  pauseExperimentService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to pause experiment: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to pause experiment, error:')
      console.log(error)
    }
  )
}

/* Resume experiment */
const resumeExperimentService = new ROSLIB.Service({
  ros: ros,
  name: '/experiment/resume',
  serviceType: 'std_srvs/Trigger',
})

export const resumeExperimentRos = (callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({}) as any

  resumeExperimentService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to resume experiment: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to resume experiment, error:')
      console.log(error)
    }
  )
}