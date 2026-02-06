import ROSLIB from '@foxglove/roslibjs'
import { ros } from './ros'
import { SessionStateValue } from 'providers/SessionProvider'
import { ExportDataType } from 'components/ExportModal'

/* Session services */
const startSessionService = new ROSLIB.Service({
  ros: ros,
  name: '/session/start',
  serviceType: 'std_srvs/Trigger',
})

const abortSessionService = new ROSLIB.Service({
  ros: ros,
  name: '/session/abort',
  serviceType: 'system_interfaces/AbortSession',
})

const exportSessionService = new ROSLIB.Service({
  ros: ros,
  name: '/session/export',
  serviceType: 'system_interfaces/ExportSession',
})

/* Session state topic */
const sessionStateTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/session/state',
  messageType: 'system_interfaces/SessionState',
})

/* Session exporter state topic */
const exporterStateTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/session_exporter/state',
  messageType: 'system_interfaces/ExporterState',
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
  callback: (success: boolean) => void
) => {
  const request = new ROSLIB.ServiceRequest({
    source: 'ui'
  }) as any

  abortSessionService.callService(
    request,
    (response: { success: boolean }) => {
      callback(response.success)
    },
    (error: any) => {
      console.log('ERROR: Failed to abort session, error:')
      console.log(error)
      callback(false)
    }
  )
}

export const subscribeToSessionState = (
  callback: (state: SessionStateValue) => void
): ROSLIB.Topic => {
  sessionStateTopic.subscribe((message: ROSLIB.Message) => {
    callback((message as any).state)
  })
  return sessionStateTopic
}

export interface ExporterState {
  state: number  // 0 = IDLE, 1 = EXPORTING, 2 = ERROR
  recording_name: string
  progress: number  // 0.0 to 1.0
}

export const subscribeToExporterState = (
  callback: (state: ExporterState) => void
): ROSLIB.Topic => {
  exporterStateTopic.subscribe((message: ROSLIB.Message) => {
    callback(message as any)
  })
  return exporterStateTopic
}

export const exportSessionRos = (
  projectName: string,
  recordingName: string,
  dataTypes: ExportDataType[],
  callback: (success: boolean, message?: string) => void
) => {
  const request = new ROSLIB.ServiceRequest({
    project_name: projectName,
    recording_name: recordingName,
    data_types: dataTypes.map(type => ({ value: type })),
  }) as any

  exportSessionService.callService(
    request,
    (response: { success: boolean }) => {
      callback(response.success, response.success ? 'Export completed successfully' : 'Export failed')
    },
    (error: any) => {
      console.log('ERROR: Failed to export session, error:')
      console.log(error)
      callback(false, 'Service call failed')
    }
  )
}