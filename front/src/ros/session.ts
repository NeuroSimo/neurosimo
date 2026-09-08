import ROSLIB from '@foxglove/roslibjs'
import { ros } from './ros'
import { SessionStateValue } from 'providers/SessionProvider'
import { ExportDataType } from 'components/ExportModal'

/* Session services */
const startSessionService = new ROSLIB.Service({
  ros: ros,
  name: '/neurosimo/session/start',
  serviceType: 'neurosimo_system_interfaces/StartSession',
})

const abortSessionService = new ROSLIB.Service({
  ros: ros,
  name: '/neurosimo/session/abort',
  serviceType: 'neurosimo_system_interfaces/AbortSession',
})

const exportSessionService = new ROSLIB.Service({
  ros: ros,
  name: '/neurosimo/session/export',
  serviceType: 'neurosimo_system_interfaces/ExportSession',
})

/* Session state topic */
const sessionStateTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/neurosimo/session/state',
  messageType: 'neurosimo_system_interfaces/SessionState',
})

/* Session exporter state topic */
const exporterStateTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/neurosimo/session_exporter/state',
  messageType: 'neurosimo_system_interfaces/ExporterState',
})

/* Session configuration, matching the fields of neurosimo_system_interfaces/SessionConfig. */
export interface SessionConfigMessage {
  subject_id: number
  notes: string
  decider_module: string
  decider_enabled: boolean
  preprocessor_module: string
  preprocessor_enabled: boolean
  presenter_module: string
  presenter_enabled: boolean
  protocol_filename: string
  runtime_parameters: string
  data_source: string
  simulator_dataset_filename: string
  simulator_start_time: number
  simulator_playback_speed: number
  replay_bag_id: string
  replay_play_preprocessed: boolean
}

export const startSessionRos = (
  config: SessionConfigMessage,
  callback: (success: boolean, message?: string) => void
) => {
  const request = new ROSLIB.ServiceRequest({ config: config }) as any

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
    source: 'ui',
    reason: 'User aborted session'
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
  callback: (state: SessionStateValue, abortReason: string) => void
): ROSLIB.Topic => {
  sessionStateTopic.subscribe((message: ROSLIB.Message) => {
    const msg = message as any
    callback(msg.state, msg.abort_reason || '')
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