import ROSLIB from 'roslib'
import { ros } from './ros'

/* Set dataset service */
const setDatasetService = new ROSLIB.Service({
  ros: ros,
  name: '/eeg_simulator/dataset/set',
  serviceType: 'project_interfaces/SetDataset',
})

export const setDatasetRos = (filename: string, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    filename: filename,
  }) as any

  setDatasetService.callService(
    request,
    (response: any) => {
      if (!response.success) {
        console.log('ERROR: Failed to set dataset: success field was false.')
      } else {
        callback()
      }
    },
    (error: any) => {
      console.log('ERROR: Failed to set dataset, error:')
      console.log(error)
    }
  )
}

/* Set start time service */
const setStartTimeService = new ROSLIB.Service({
  ros: ros,
  name: '/eeg_simulator/start_time/set',
  serviceType: 'project_interfaces/SetStartTime',
})

export const setStartTimeRos = (startTime: number, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    start_time: startTime,
  }) as any

  setStartTimeService.callService(
    request,
    (response: any) => {
      if (!response.success) {
        console.log('ERROR: Failed to set start time: success field was false.')
      } else {
        callback()
      }
    },
    (error: any) => {
      console.log('ERROR: Failed to set start time, error:')
      console.log(error)
    }
  )
}
