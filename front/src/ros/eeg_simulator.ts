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
    (response: any) => {
      if (!response.success) {
        console.log('ERROR: Failed to set loop: success field was false.')
      } else {
        callback()
      }
    },
    (error: any) => {
      console.log('ERROR: Failed to set loop, error:')
      console.log(error)
    }
  )
}

/* Start simulator service */
const startSimulatorService = new ROSLIB.Service({
  ros: ros,
  name: '/eeg_simulator/start',
  serviceType: 'std_srvs/Trigger',
})

export const startSimulatorRos = (callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({}) as any

  startSimulatorService.callService(
    request,
    (response: any) => {
      if (!response.success) {
        console.log('ERROR: Failed to start simulator: success field was false.')
      } else {
        callback()
      }
    },
    (error: any) => {
      console.log('ERROR: Failed to start simulator, error:')
      console.log(error)
    }
  )
}

/* Stop simulator service */
const stopSimulatorService = new ROSLIB.Service({
  ros: ros,
  name: '/eeg_simulator/stop',
  serviceType: 'std_srvs/Trigger',
})

export const stopSimulatorRos = (callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({}) as any

  stopSimulatorService.callService(
    request,
    (response: any) => {
      if (!response.success) {
        console.log('ERROR: Failed to stop simulator: success field was false.')
      } else {
        callback()
      }
    },
    (error: any) => {
      console.log('ERROR: Failed to stop simulator, error:')
      console.log(error)
    }
  )
}
