import ROSLIB from '@foxglove/roslibjs'
import { ros } from './ros'

const setSystemConfigService = new ROSLIB.Service({
  ros: ros,
  name: '/neurosimo/system_configurator/config/set',
  serviceType: 'neurosimo_system_interfaces/SetSystemConfig',
})

/* System configuration, matching the fields of neurosimo_system_interfaces/SystemConfig. */
export interface SystemConfigMessage {
  active_project: string
  eeg_port: number
  eeg_device: string
  turbolink_sampling_frequency: number
  turbolink_eeg_channel_count: number
  maximum_dropped_samples: number
  enable_labjack: boolean
  maximum_loopback_latency: number
  maximum_timing_error: number
  trigger_to_pulse_delay: number
  disk_warning_threshold: string
  disk_error_threshold: string
  locale: string
}

/* The service is a full replace: the request must carry the complete configuration. */
export const setSystemConfigRos = (
  config: SystemConfigMessage,
  callback: (success: boolean, message?: string) => void
) => {
  const request = new ROSLIB.ServiceRequest({ config: config }) as any

  setSystemConfigService.callService(
    request,
    (response: { success: boolean; message: string }) => {
      callback(response.success, response.message)
    },
    (error: any) => {
      console.log('ERROR: Failed to set system config, error:')
      console.log(error)
      callback(false, 'Service call failed')
    }
  )
}
