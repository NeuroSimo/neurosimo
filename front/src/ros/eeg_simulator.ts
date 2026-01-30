import ROSLIB from '@foxglove/roslibjs'
import { ros } from './ros'

export interface DatasetInfo extends ROSLIB.Message {
  json_filename: string
  name: string
  data_filename: string
  sampling_frequency: number
  num_eeg_channels: number
  num_emg_channels: number
  duration: number
  loop: boolean
  pulse_count: number
}

/* Get dataset info service */
const getDatasetInfoService = new ROSLIB.Service({
  ros: ros,
  name: '/eeg_simulator/dataset/get_info',
  serviceType: 'project_interfaces/GetDatasetInfo',
})

export const getDatasetInfoRos = (
  filename: string,
  callback: (datasetInfo: DatasetInfo | null) => void
) => {
  const request = new ROSLIB.ServiceRequest({
    filename: filename
  }) as any

  getDatasetInfoService.callService(
    request,
    (response: { dataset_info: DatasetInfo; success: boolean }) => {
      if (!response.success) {
        console.log('ERROR: Failed to get dataset info: success field was false.')
        callback(null)
      } else {
        callback(response.dataset_info)
      }
    },
    (error) => {
      console.log('ERROR: Failed to get dataset info, error:')
      console.log(error)
      callback(null)
    }
  )
}