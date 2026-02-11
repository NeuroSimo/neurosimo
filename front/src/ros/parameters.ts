import ROSLIB from '@foxglove/roslibjs'
import { ros } from './ros'

// Type definitions for ROS parameter values
type ParameterValue = boolean | number | string

interface Parameter {
  name: string
  value: ParameterValue
}

// Parameters that should always be sent as DOUBLE type, even if they are whole numbers
const ALWAYS_DOUBLE_PARAMS = new Set([
  'minimum_intertrial_interval',
  'maximum_loopback_latency',
  'maximum_timing_offset',
  'trigger_to_pulse_delay',
])

/**
 * Convert a TypeScript value to ROS2 ParameterValue structure
 * @param value The value to convert
 * @param paramName Optional parameter name to determine type for numbers
 * @returns ROS2 ParameterValue object
 */
const toParameterValue = (value: ParameterValue, paramName?: string): any => {
  if (typeof value === 'boolean') {
    return {
      type: 1, // PARAMETER_BOOL
      bool_value: value
    }
  } else if (typeof value === 'number') {
    // Check if this parameter should always be a DOUBLE
    const forceDouble = paramName && ALWAYS_DOUBLE_PARAMS.has(paramName)
    
    // Check if it's an integer or float
    if (!forceDouble && Number.isInteger(value)) {
      return {
        type: 2, // PARAMETER_INTEGER
        integer_value: value
      }
    } else {
      return {
        type: 3, // PARAMETER_DOUBLE
        double_value: value
      }
    }
  } else if (typeof value === 'string') {
    return {
      type: 4, // PARAMETER_STRING
      string_value: value
    }
  }
  throw new Error(`Unsupported parameter value type: ${typeof value}`)
}

/**
 * Extract the actual value from a ROS2 ParameterValue structure
 * @param paramValue ROS2 ParameterValue object
 * @returns TypeScript value or undefined if unsupported type
 */
export const extractParameterValue = (paramValue: {
  type: number
  bool_value?: boolean
  integer_value?: number
  double_value?: number
  string_value?: string
}): boolean | number | string | undefined => {
  switch (paramValue.type) {
    case 1: // PARAMETER_BOOL
      return paramValue.bool_value
    case 2: // PARAMETER_INTEGER
      return paramValue.integer_value
    case 3: // PARAMETER_DOUBLE
      return paramValue.double_value
    case 4: // PARAMETER_STRING
      return paramValue.string_value
    default:
      console.warn(`Unknown parameter type: ${paramValue.type}`)
      return undefined
  }
}

/* Set parameters service on session_configurator node */
const sessionConfiguratorSetParametersService = new ROSLIB.Service({
  ros: ros,
  name: '/session_configurator/set_parameters',
  serviceType: 'rcl_interfaces/SetParameters',
})

/* Set parameters service on global_configurator node */
const globalConfiguratorSetParametersService = new ROSLIB.Service({
  ros: ros,
  name: '/global_configurator/set_parameters',
  serviceType: 'rcl_interfaces/SetParameters',
})

/**
 * Set a single ROS parameter on a node
 * @param name Parameter name (e.g., 'decider.module', 'active_project')
 * @param value Parameter value (boolean, number, or string)
 * @param callback Callback function called on success
 * @param nodeName Node name ('session_configurator' or 'global_configurator'), defaults to 'session_configurator'
 */
export const setParameterRos = (
  name: string, 
  value: ParameterValue, 
  callback: () => void,
  nodeName: 'session_configurator' | 'global_configurator' = 'session_configurator'
) => {
  const parameter = {
    name: name,
    value: toParameterValue(value, name)
  }

  const request = new ROSLIB.ServiceRequest({
    parameters: [parameter]
  }) as any

  const service = nodeName === 'global_configurator' 
    ? globalConfiguratorSetParametersService 
    : sessionConfiguratorSetParametersService

  service.callService(
    request,
    (response: any) => {
      if (!response.results || !response.results[0] || !response.results[0].successful) {
        console.log(`ERROR: Failed to set parameter ${name}: parameter set was not successful.`)
        if (response.results && response.results[0]) {
          console.log('Reason:', response.results[0].reason)
        }
      } else {
        callback()
      }
    },
    (error: any) => {
      console.log(`ERROR: Failed to set parameter ${name}, error:`, error)
    }
  )
}

/**
 * Set multiple ROS parameters on a node
 * @param parameters Array of parameter objects with name and value
 * @param callback Callback function called on success
 * @param nodeName Node name ('session_configurator' or 'global_configurator'), defaults to 'session_configurator'
 */
export const setParametersRos = (
  parameters: Parameter[], 
  callback: () => void,
  nodeName: 'session_configurator' | 'global_configurator' = 'session_configurator'
) => {
  const formattedParameters = parameters.map(param => ({
    name: param.name,
    value: toParameterValue(param.value, param.name)
  }))

  const request = new ROSLIB.ServiceRequest({
    parameters: formattedParameters
  }) as any

  const service = nodeName === 'global_configurator' 
    ? globalConfiguratorSetParametersService 
    : sessionConfiguratorSetParametersService

  service.callService(
    request,
    (response: any) => {
      const failedParams = response.results.filter((result: any, index: number) => {
        return !result.successful
      })

      if (failedParams.length > 0) {
        console.log('ERROR: Failed to set some parameters:')
        failedParams.forEach((result: any, index: number) => {
          console.log(`  ${parameters[index].name}: ${result.reason}`)
        })
      } else {
        callback()
      }
    },
    (error: any) => {
      console.log('ERROR: Failed to set parameters, error:', error)
    }
  )
}

/* Get parameters service on session_configurator node */
const sessionConfiguratorGetParametersService = new ROSLIB.Service({
  ros: ros,
  name: '/session_configurator/get_parameters',
  serviceType: 'rcl_interfaces/GetParameters',
})

/* Get parameters service on global_configurator node */
const globalConfiguratorGetParametersService = new ROSLIB.Service({
  ros: ros,
  name: '/global_configurator/get_parameters',
  serviceType: 'rcl_interfaces/GetParameters',
})

/**
 * Get current ROS parameters from a node
 * @param names Array of parameter names to fetch
 * @param nodeName Node name ('session_configurator' or 'global_configurator'), defaults to 'session_configurator'
 * @returns Promise resolving to parameter values
 */
export const getParametersRos = (
  names: string[],
  nodeName: 'session_configurator' | 'global_configurator' = 'session_configurator'
): Promise<Map<string, boolean | number | string>> => {
  return new Promise((resolve, reject) => {
    const request = new ROSLIB.ServiceRequest({
      names: names
    }) as any

    const service = nodeName === 'global_configurator'
      ? globalConfiguratorGetParametersService
      : sessionConfiguratorGetParametersService

    service.callService(
      request,
      (response: any) => {
        const parameterMap = new Map<string, boolean | number | string>()

        response.values.forEach((paramValue: any, index: number) => {
          const value = extractParameterValue(paramValue)
          if (value !== undefined) {
            parameterMap.set(names[index], value)
          }
        })

        resolve(parameterMap)
      },
      (error: any) => {
        console.log('ERROR: Failed to get parameters, error:', error)
        reject(error)
      }
    )
  })
}
