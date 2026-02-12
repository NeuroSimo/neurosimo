import ROSLIB from '@foxglove/roslibjs'
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
  notes: string
  protocol_filename: string
  data_source: string
  simulator_dataset_filename: string
  simulator_start_time: number
  replay_bag_id: string
  replay_play_preprocessed: boolean
  start_time: string
  end_time: string
  git_commit: string
  git_state: string
  version: string
  data_source_fingerprint: number
  preprocessor_fingerprint: number
  decision_fingerprint: number
  exported: boolean
  export_directory: string
}

/* Get recording info service */
const getRecordingInfoService = new ROSLIB.Service({
  ros: ros,
  name: '/recording_manager/recording/get_info',
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

/* Delete recording service */
const deleteRecordingService = new ROSLIB.Service({
  ros: ros,
  name: '/recording_manager/recording/delete',
  serviceType: 'project_interfaces/DeleteRecording',
})

export const deleteRecordingRos = (
  bagId: string,
  callback: (success: boolean) => void
) => {
  const request = new ROSLIB.ServiceRequest({
    bag_id: bagId
  }) as any

  deleteRecordingService.callService(
    request,
    (response: { success: boolean }) => {
      if (!response.success) {
        console.log('ERROR: Failed to delete recording: success field was false.')
        callback(false)
      } else {
        callback(true)
      }
    },
    (error) => {
      console.log('ERROR: Failed to delete recording, error:')
      console.log(error)
      callback(false)
    }
  )
}
