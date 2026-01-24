import ROSLIB from 'roslib'
import { ros } from './ros'

export enum SessionStage {
  STOPPED = 0,
  INITIALIZING = 1,
  RUNNING = 2,
  FINALIZING = 3,
}

export interface SessionState extends ROSLIB.Message {
  is_running: boolean
  stage: SessionStage
}

/* Session services */
const startSessionService = new ROSLIB.Service({
  ros: ros,
  name: '/session/start',
  serviceType: 'std_srvs/Trigger',
})

const abortSessionService = new ROSLIB.Service({
  ros: ros,
  name: '/session/abort',
  serviceType: 'std_srvs/Trigger',
})

/* Session state topic */
const sessionStateTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/session/state',
  messageType: 'system_interfaces/SessionState',
})

export const startSessionRos = (
  callback: (success: boolean, message?: string) => void
) => {
  const request = new ROSLIB.ServiceRequest({}) as any

  startSessionService.callService(
    request,
    (response: { success: boolean; message: string }) => {
      callback(response.success, response.message)
    },
    (error: any) => {
      console.log('ERROR: Failed to start session, error:')
      console.log(error)
      callback(false, 'Service call failed')
    }
  )
}

export const abortSessionRos = (
  callback: (success: boolean, message?: string) => void
) => {
  const request = new ROSLIB.ServiceRequest({}) as any

  abortSessionService.callService(
    request,
    (response: { success: boolean; message: string }) => {
      callback(response.success, response.message)
    },
    (error: any) => {
      console.log('ERROR: Failed to abort session, error:')
      console.log(error)
      callback(false, 'Service call failed')
    }
  )
}

export const subscribeToSessionState = (
  callback: (state: SessionState) => void
): ROSLIB.Topic => {
  sessionStateTopic.subscribe((message: ROSLIB.Message) => {
    callback(message as SessionState)
  })
  return sessionStateTopic
}