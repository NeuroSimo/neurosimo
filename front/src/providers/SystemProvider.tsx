import React, { useState, useEffect, ReactNode } from 'react'
import { Topic, Message } from 'roslib'

import { ros } from 'ros/ros'

/* System errors */
export const StartupError = {
  NO_ERROR: 0,
  UART_INITIALIZATION_ERROR: 1,
  BOARD_STARTUP_ERROR: 2,
  BOARD_STATUS_MESSAGE_ERROR: 3,
  SAFETY_MONITOR_ERROR: 4,
  DISCHARGE_CONTROLLER_ERROR: 5,
  CHARGER_ERROR: 6,
  SENSORBOARD_ERROR: 7,
  DISCHARGE_CONTROLLER_VOLTAGE_ERROR: 8,
  CHARGER_VOLTAGE_ERROR: 9,
  IGBT_FEEDBACK_ERROR: 10,
  TEMPERATURE_SENSOR_PRESENCE_ERROR: 11,
  COIL_MEMORY_PRESENCE_ERROR: 12,
}

export interface SystemError {
  heartbeat_error: boolean
  latched_fault_error: boolean
  powersupply_error: boolean
  safety_bus_error: boolean
  coil_error: boolean
  emergency_button_error: boolean
  door_error: boolean
  charger_overvoltage_error: boolean
  charger_overtemperature_error: boolean
  monitored_voltage_over_maximum_error: boolean
  patient_safety_error: boolean
  device_safety_error: boolean
  charger_powerup_error: boolean
  opto_error: boolean
  charger_power_enabled_twice_error: boolean
}

export interface ChannelError {
  overvoltage_error: boolean
  emergency_discharge_backup_power_error: boolean
  safety_bus_error: boolean
  powersupply_error: boolean
  safety_bus_startup_error: boolean
  acceptable_voltage_not_reached_startup_error: boolean
  maximum_safe_voltage_exceeded_startup_error: boolean
}

/* Session */
export const SessionState = {
  STOPPED: 0,
  STARTING: 1,
  STARTED: 2,
  STOPPING: 3,
}

export const HumanReadableSessionState = {
  STOPPED: 'Stopped',
  STARTING: 'Starting...',
  STARTED: 'Started',
  STOPPING: 'Stopping...',
}

export interface Session extends Message {
  state: SessionState
  time: number
}

/* System state */
export const DeviceState = {
  NOT_OPERATIONAL: 0,
  STARTUP: 1,
  OPERATIONAL: 2,
  SHUTDOWN: 3,
}

export const HumanReadableDeviceState = {
  NOT_OPERATIONAL: 'Not operational',
  STARTUP: 'Starting up...',
  OPERATIONAL: 'Operational',
  SHUTDOWN: 'Shutting down...',
}

export interface SystemState extends Message {
  channel_states: ChannelState[]

  system_error_cumulative: SystemError
  system_error_current: SystemError
  system_error_emergency: SystemError

  startup_error: Error

  device_state: DeviceState
}

interface Error {
  value: number
}

export interface SessionState {
  value: number
}

export interface DeviceState {
  value: number
}

export interface ChannelState {
  channel_index: number
  voltage: number
  temperature: number
  pulse_count: number
  channel_error: ChannelError
}

/* Context */
interface SystemContextType {
  systemState: SystemState | null
  session: Session | null
}

const defaultSystemState: SystemContextType = {
  systemState: null,
  session: null,
}

export const SystemContext = React.createContext<SystemContextType>(defaultSystemState)

interface SystemProviderProps {
  children: ReactNode
}

export const SystemProvider: React.FC<SystemProviderProps> = ({ children }) => {
  const [systemState, setSystemState] = useState<SystemState | null>(null)
  const [session, setSession] = useState<Session | null>(null)

  useEffect(() => {
    /* Subscriber for system state. */
    const systemStateSubscriber = new Topic<SystemState>({
      ros: ros,
      name: '/mtms_device/system_state',
      messageType: 'mtms_device_interfaces/SystemState',
    })

    systemStateSubscriber.subscribe((message) => {
      setSystemState(message)
    })

    /* Subscriber for session. */
    const sessionSubscriber = new Topic<Session>({
      ros: ros,
      name: '/system/session',
      messageType: 'system_interfaces/Session',
    })

    sessionSubscriber.subscribe((message) => {
      setSession(message)
    })

    /* Unsubscribers */
    return () => {
      systemStateSubscriber.unsubscribe()
      sessionSubscriber.unsubscribe()
    }
  }, [])

  return <SystemContext.Provider value={{ systemState, session }}>{children}</SystemContext.Provider>
}
