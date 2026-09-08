import React, { useState, useEffect, useRef, ReactNode, createContext, useContext } from 'react'
import { Topic } from '@foxglove/roslibjs'

import { ros } from 'ros/ros'
import { setSystemConfigRos, SystemConfigMessage } from 'ros/systemConfig'
import { FilenameList } from './ModuleListProvider'

/* The camelCase mirror of neurosimo_system_interfaces/SystemConfig. */
export interface SystemConfigValues {
  activeProject: string
  eegPort: number
  eegDevice: string
  turbolinkSamplingFrequency: number
  turbolinkEegChannelCount: number
  maximumDroppedSamples: number
  enableLabjack: boolean
  maximumLoopbackLatency: number
  maximumTimingError: number
  triggerToPulseDelay: number
  diskWarningThreshold: string
  diskErrorThreshold: string
  locale: string
}

interface SystemConfigContextType extends SystemConfigValues {
  projects: string[]
  setActiveProject: (project: string, callback?: () => void) => Promise<void>
  setSystemConfig: (config: Partial<SystemConfigValues>, callback?: () => void) => Promise<void>
}

// ESLint disable for intentionally empty functions used as defaults
/* eslint-disable @typescript-eslint/no-empty-function */
const asyncNoop = async () => {}
/* eslint-enable @typescript-eslint/no-empty-function */

const emptySystemConfig: SystemConfigValues = {
  activeProject: '',
  eegPort: 0,
  eegDevice: '',
  turbolinkSamplingFrequency: 0,
  turbolinkEegChannelCount: 0,
  maximumDroppedSamples: 0,
  enableLabjack: false,
  maximumLoopbackLatency: 0,
  maximumTimingError: 0,
  triggerToPulseDelay: 0,
  diskWarningThreshold: '',
  diskErrorThreshold: '',
  locale: '',
}

const defaultSystemConfigState: SystemConfigContextType = {
  ...emptySystemConfig,
  projects: [],
  setActiveProject: asyncNoop,
  setSystemConfig: asyncNoop,
}

const toMessage = (config: SystemConfigValues): SystemConfigMessage => ({
  active_project: config.activeProject,
  eeg_port: config.eegPort,
  eeg_device: config.eegDevice,
  turbolink_sampling_frequency: config.turbolinkSamplingFrequency,
  turbolink_eeg_channel_count: config.turbolinkEegChannelCount,
  maximum_dropped_samples: config.maximumDroppedSamples,
  enable_labjack: config.enableLabjack,
  maximum_loopback_latency: config.maximumLoopbackLatency,
  maximum_timing_error: config.maximumTimingError,
  trigger_to_pulse_delay: config.triggerToPulseDelay,
  disk_warning_threshold: config.diskWarningThreshold,
  disk_error_threshold: config.diskErrorThreshold,
  locale: config.locale,
})

export const SystemConfigContext = createContext<SystemConfigContextType>(defaultSystemConfigState)

interface SystemConfigProviderProps {
  children: ReactNode
}

export const SystemConfigProvider: React.FC<SystemConfigProviderProps> = ({ children }) => {
  const [systemConfig, setSystemConfigState] = useState<SystemConfigValues>(emptySystemConfig)
  const [projects, setProjects] = useState<string[]>([])

  /* The set service is a full replace, so partial updates have to be merged against the
     latest known configuration. Keep it in a ref so the setters never close over stale state. */
  const systemConfigRef = useRef<SystemConfigValues>(systemConfig)
  systemConfigRef.current = systemConfig

  /* Until the latched config has arrived there is nothing to merge against, and sending
     would replace the stored configuration with empty values. */
  const hasReceivedConfig = useRef(false)

  useEffect(() => {
    /* Subscriber for system config topic (latched). */
    const systemConfigSubscriber = new Topic({
      ros: ros,
      name: '/neurosimo/system_configurator/config',
      messageType: 'neurosimo_system_interfaces/SystemConfig',
      queue_size: 1,
    })

    systemConfigSubscriber.subscribe((message: ROSLIB.Message) => {
      const msg = message as any
      hasReceivedConfig.current = true
      setSystemConfigState({
        activeProject: msg.active_project,
        eegPort: msg.eeg_port,
        eegDevice: msg.eeg_device,
        turbolinkSamplingFrequency: msg.turbolink_sampling_frequency,
        turbolinkEegChannelCount: msg.turbolink_eeg_channel_count,
        maximumDroppedSamples: msg.maximum_dropped_samples,
        enableLabjack: msg.enable_labjack,
        maximumLoopbackLatency: msg.maximum_loopback_latency,
        maximumTimingError: msg.maximum_timing_error,
        triggerToPulseDelay: msg.trigger_to_pulse_delay,
        diskWarningThreshold: msg.disk_warning_threshold,
        diskErrorThreshold: msg.disk_error_threshold,
        locale: msg.locale,
      })
    })

    /* Subscriber for the available projects list (latched). */
    const projectListSubscriber = new Topic<FilenameList>({
      ros: ros,
      name: '/neurosimo/system_configurator/projects',
      messageType: 'neurosimo_project_interfaces/FilenameList',
    })

    projectListSubscriber.subscribe((message) => {
      setProjects(message.filenames)
    })

    /* Cleanup */
    return () => {
      systemConfigSubscriber.unsubscribe()
      projectListSubscriber.unsubscribe()
    }
  }, [])

  const applyUpdate = async (
    update: Partial<SystemConfigValues>,
    callback?: () => void
  ): Promise<void> => {
    if (!hasReceivedConfig.current) {
      console.log('ERROR: Refusing to set system config before the current one has been received')
      return
    }

    const merged = { ...systemConfigRef.current, ...update }

    setSystemConfigRos(toMessage(merged), (success, message) => {
      if (!success) {
        console.log(`ERROR: Failed to set system config: ${message}`)
      }
      callback?.()
    })
  }

  const setActiveProject = async (project: string, callback?: () => void): Promise<void> =>
    applyUpdate({ activeProject: project }, callback)

  return (
    <SystemConfigContext.Provider
      value={{
        ...systemConfig,
        projects: projects,
        setActiveProject,
        setSystemConfig: applyUpdate,
      }}
    >
      {children}
    </SystemConfigContext.Provider>
  )
}

export const useSystemConfig = () => {
  const context = useContext(SystemConfigContext)
  if (!context) {
    throw new Error('useSystemConfig must be used within a SystemConfigProvider')
  }
  return context
}
