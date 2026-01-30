import ROSLIB from '@foxglove/roslibjs'
import { ros } from './ros'

export interface ProtocolStageInfo extends ROSLIB.Message {
  name: string
  trials: number
  notes: string
}

export interface ProtocolRestInfo extends ROSLIB.Message {
  notes: string
}

export interface ProtocolElementInfo extends ROSLIB.Message {
  type: number  // 0 = STAGE, 1 = REST
  stage: ProtocolStageInfo
  rest: ProtocolRestInfo
}

export interface ProtocolInfo extends ROSLIB.Message {
  yaml_filename: string
  name: string
  description: string
  elements: ProtocolElementInfo[]
}

export const PROTOCOL_ELEMENT_TYPE = {
  STAGE: 0,
  REST: 1,
}

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

/* Get protocol info service */
const getProtocolInfoService = new ROSLIB.Service({
  ros: ros,
  name: '/experiment_coordinator/protocol/get_info',
  serviceType: 'pipeline_interfaces/GetProtocolInfo',
})

export const getProtocolInfoRos = (
  projectName: string,
  filename: string,
  callback: (protocolInfo: ProtocolInfo | null) => void
) => {
  const request = new ROSLIB.ServiceRequest({
    project_name: projectName,
    filename: filename
  }) as any

  getProtocolInfoService.callService(
    request,
    (response: { protocol_info: ProtocolInfo; success: boolean }) => {
      if (!response.success) {
        console.log('ERROR: Failed to get protocol info: success field was false.')
        callback(null)
      } else {
        callback(response.protocol_info)
      }
    },
    (error) => {
      console.log('ERROR: Failed to get protocol info, error:')
      console.log(error)
      callback(null)
    }
  )
}