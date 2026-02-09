import React, { useState, useEffect, ReactNode, createContext, useContext } from 'react'
import { Topic } from '@foxglove/roslibjs'

import { ros } from 'ros/ros'

interface GlobalConfigContextType {
  activeProject: string
  eegPort: number
  eegDevice: string
  turbolinkSamplingFrequency: number
  turbolinkEegChannelCount: number
  maximumDroppedSamples: number
  simulateLabjack: boolean
  minimumIntertrialInterval: number
  maximumLoopbackLatency: number
  maximumTimingError: number
  diskWarningThreshold: string
  diskErrorThreshold: string
  locale: string
  setActiveProject: (project: string, callback?: () => void) => Promise<void>
  setGlobalConfig: (config: Partial<Omit<GlobalConfigContextType, 'setActiveProject' | 'setGlobalConfig'>>, callback?: () => void) => Promise<void>
}

// ESLint disable for intentionally empty functions used as defaults
/* eslint-disable @typescript-eslint/no-empty-function */
const asyncNoop = async () => {}
/* eslint-enable @typescript-eslint/no-empty-function */

const defaultGlobalConfigState: GlobalConfigContextType = {
  activeProject: '',
  eegPort: 0,
  eegDevice: '',
  turbolinkSamplingFrequency: 0,
  turbolinkEegChannelCount: 0,
  maximumDroppedSamples: 0,
  simulateLabjack: false,
  minimumIntertrialInterval: 0,
  maximumLoopbackLatency: 0,
  maximumTimingError: 0,
  diskWarningThreshold: '',
  diskErrorThreshold: '',
  locale: '',
  setActiveProject: asyncNoop,
  setGlobalConfig: asyncNoop,
}

export const GlobalConfigContext = createContext<GlobalConfigContextType>(defaultGlobalConfigState)

interface GlobalConfigProviderProps {
  children: ReactNode
}

export const GlobalConfigProvider: React.FC<GlobalConfigProviderProps> = ({ children }) => {
  const [globalConfig, setGlobalConfig] = useState<{
    activeProject: string
    eegPort: number
    eegDevice: string
    turbolinkSamplingFrequency: number
    turbolinkEegChannelCount: number
    maximumDroppedSamples: number
    simulateLabjack: boolean
    minimumIntertrialInterval: number
    maximumLoopbackLatency: number
    maximumTimingError: number
    diskWarningThreshold: string
    diskErrorThreshold: string
    locale: string
  }>({
    activeProject: '',
    eegPort: 0,
    eegDevice: '',
    turbolinkSamplingFrequency: 0,
    turbolinkEegChannelCount: 0,
    maximumDroppedSamples: 0,
    simulateLabjack: false,
    minimumIntertrialInterval: 0,
    maximumLoopbackLatency: 0,
    maximumTimingError: 0,
    diskWarningThreshold: '',
    diskErrorThreshold: '',
    locale: '',
  })

  useEffect(() => {
    /* Subscriber for global config topic (latched). */
    const globalConfigSubscriber = new Topic({
      ros: ros,
      name: '/global_configurator/config',
      messageType: 'system_interfaces/GlobalConfig',
      queue_size: 1,
    })

    globalConfigSubscriber.subscribe((message: ROSLIB.Message) => {
      const msg = message as any
      setGlobalConfig({
        activeProject: msg.active_project,
        eegPort: msg.eeg_port,
        eegDevice: msg.eeg_device,
        turbolinkSamplingFrequency: msg.turbolink_sampling_frequency,
        turbolinkEegChannelCount: msg.turbolink_eeg_channel_count,
        maximumDroppedSamples: msg.maximum_dropped_samples,
        simulateLabjack: msg.simulate_labjack,
        minimumIntertrialInterval: msg.minimum_intertrial_interval,
        maximumLoopbackLatency: msg.maximum_loopback_latency,
        maximumTimingError: msg.maximum_timing_error,
        diskWarningThreshold: msg.disk_warning_threshold,
        diskErrorThreshold: msg.disk_error_threshold,
        locale: msg.locale,
      })
    })

    /* Cleanup */
    return () => {
      globalConfigSubscriber.unsubscribe()
    }
  }, [])

  // Convenience setter - use dynamic import to avoid circular dependencies
  const noop = () => {} // eslint-disable-line @typescript-eslint/no-empty-function

  const setActiveProject = async (project: string, callback?: () => void): Promise<void> => {
    const { setParameterRos } = await import('../ros/parameters')
    setParameterRos('active_project', project, callback || noop, 'global_configurator')
  }

  const setGlobalConfigParams = async (
    config: Partial<Omit<GlobalConfigContextType, 'setActiveProject' | 'setGlobalConfig'>>,
    callback?: () => void
  ): Promise<void> => {
    const { setParametersRos } = await import('../ros/parameters')
    
    const parameters: Array<{ name: string; value: boolean | number | string }> = []
    
    if (config.activeProject !== undefined) parameters.push({ name: 'active_project', value: config.activeProject })
    if (config.eegPort !== undefined) parameters.push({ name: 'eeg_port', value: config.eegPort })
    if (config.eegDevice !== undefined) parameters.push({ name: 'eeg_device', value: config.eegDevice })
    if (config.turbolinkSamplingFrequency !== undefined) parameters.push({ name: 'turbolink_sampling_frequency', value: config.turbolinkSamplingFrequency })
    if (config.turbolinkEegChannelCount !== undefined) parameters.push({ name: 'turbolink_eeg_channel_count', value: config.turbolinkEegChannelCount })
    if (config.maximumDroppedSamples !== undefined) parameters.push({ name: 'maximum_dropped_samples', value: config.maximumDroppedSamples })
    if (config.simulateLabjack !== undefined) parameters.push({ name: 'simulate_labjack', value: config.simulateLabjack })
    if (config.minimumIntertrialInterval !== undefined) parameters.push({ name: 'minimum_intertrial_interval', value: config.minimumIntertrialInterval })
    if (config.maximumLoopbackLatency !== undefined) parameters.push({ name: 'maximum_loopback_latency', value: config.maximumLoopbackLatency })
    if (config.maximumTimingError !== undefined) parameters.push({ name: 'maximum_timing_error', value: config.maximumTimingError })
    if (config.diskWarningThreshold !== undefined) parameters.push({ name: 'disk_warning_threshold', value: config.diskWarningThreshold })
    if (config.diskErrorThreshold !== undefined) parameters.push({ name: 'disk_error_threshold', value: config.diskErrorThreshold })
    if (config.locale !== undefined) parameters.push({ name: 'locale', value: config.locale })
    
    setParametersRos(parameters, callback || noop, 'global_configurator')
  }

  return (
    <GlobalConfigContext.Provider
      value={{
        activeProject: globalConfig.activeProject,
        eegPort: globalConfig.eegPort,
        eegDevice: globalConfig.eegDevice,
        turbolinkSamplingFrequency: globalConfig.turbolinkSamplingFrequency,
        turbolinkEegChannelCount: globalConfig.turbolinkEegChannelCount,
        maximumDroppedSamples: globalConfig.maximumDroppedSamples,
        simulateLabjack: globalConfig.simulateLabjack,
        minimumIntertrialInterval: globalConfig.minimumIntertrialInterval,
        maximumLoopbackLatency: globalConfig.maximumLoopbackLatency,
        maximumTimingError: globalConfig.maximumTimingError,
        diskWarningThreshold: globalConfig.diskWarningThreshold,
        diskErrorThreshold: globalConfig.diskErrorThreshold,
        locale: globalConfig.locale,
        setActiveProject,
        setGlobalConfig: setGlobalConfigParams,
      }}
    >
      {children}
    </GlobalConfigContext.Provider>
  )
}

export const useGlobalConfig = () => {
  const context = useContext(GlobalConfigContext)
  if (!context) {
    throw new Error('useGlobalConfig must be used within a GlobalConfigProvider')
  }
  return context
}
