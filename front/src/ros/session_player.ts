import ROSLIB from 'roslib'
import { ros } from './ros'

export interface RecordingInfo extends ROSLIB.Message {
  json_filename: string
  num_eeg_channels: number
  num_emg_channels: number
  sampling_frequency: number
  duration: number
  preprocessor_module: string
  decider_module: string
  presenter_module: string
  preprocessor_enabled: boolean
  decider_enabled: boolean
  presenter_enabled: boolean
  project_name: string
  subject_id: string
  protocol_name: string
  start_time: string
  end_time: string
  git_commit: string
  git_state: string
  version: string
}

/* Get recording info service */
const getRecordingInfoService = new ROSLIB.Service({
  ros: ros,
  name: '/session_player/recording/get_info',
  serviceType: 'project_interfaces/GetRecordingInfo',
})

export const getRecordingInfoRos = (
  filename: string,
  callback: (recordingInfo: RecordingInfo | null) => void
) => {
  const request = new ROSLIB.ServiceRequest({
    filename: filename
  }) as any

  getRecordingInfoService.callService(
    request,
    (response: { recording_info: RecordingInfo; success: boolean }) => {
      if (!response.success) {
        console.log('ERROR: Failed to get recording info: success field was false.')
        callback(null)
      } else {
        callback(response.recording_info)
      }
    },
    (error) => {
      console.log('ERROR: Failed to get recording info, error:')
      console.log(error)
      callback(null)
    }
  )
}
