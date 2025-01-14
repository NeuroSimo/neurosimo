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
  serviceType: 'project_interfaces/SetPlayback',
})

export const setPlaybackRos = (playback: boolean, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    playback: playback,
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

/* Set loop service */
const setLoopService = new ROSLIB.Service({
  ros: ros,
  name: '/eeg_simulator/loop/set',
  serviceType: 'project_interfaces/SetLoop',
})

export const setLoopRos = (loop: boolean, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    loop: loop,
  }) as any

  setLoopService.callService(
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

/* Set record data service */
const setRecordDataService = new ROSLIB.Service({
  ros: ros,
  name: '/eeg_recorder/record_data/set',
  serviceType: 'project_interfaces/SetRecordData',
})

export const setRecordDataRos = (recordData: boolean, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    record_data: recordData,
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
