import ROSLIB from 'roslib'
import { ros } from '../ros'

const startDeviceService = new ROSLIB.Service({
  ros: ros,
  name: '/mtms_device/start_device',
  serviceType: 'mtms_device_interfaces/StartDevice',
})

const stopDeviceService = new ROSLIB.Service({
  ros: ros,
  name: '/mtms_device/stop_device',
  serviceType: 'mtms_device_interfaces/StopDevice',
})

export const startDevice = () => {
  const request = new ROSLIB.ServiceRequest({})
  startDeviceService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to start session')
      }
    },
    (error) => {
      console.log('ERROR: Failed to start session')
      console.error(error)
    },
  )
}

export const stopDevice = () => {
  const request = new ROSLIB.ServiceRequest({})
  stopDeviceService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to stop experiment')
      }
    },
    (error) => {
      console.log('ERROR: Failed to stop experiment')
      console.error(error)
    },
  )
}
