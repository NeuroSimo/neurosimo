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
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to set dataset: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to set dataset, error:')
      console.log(error)
    }
  )
}

/* Set playback service */
const setPlaybackService = new ROSLIB.Service({
  ros: ros,
  name: '/eeg_simulator/playback/set',
  serviceType: 'std_srvs/SetBool',
})

export const setPlaybackRos = (playback: boolean, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    data: playback,
  }) as any

  setPlaybackService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to set playback: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to set playback, error:')
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
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to set start time: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to set start time, error:')
      console.log(error)
    }
  )
}

/* Set record data service */
const setRecordDataService = new ROSLIB.Service({
  ros: ros,
  name: '/eeg_recorder/record_data/set',
  serviceType: 'std_srvs/SetBool',
})

export const setRecordDataRos = (recordData: boolean, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    data: recordData,
  }) as any

  setRecordDataService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to set loop: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to set loop, error:')
      console.log(error)
    }
  )
}
