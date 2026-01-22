import ROSLIB from 'roslib'
import { ros } from './ros'

// Type definitions for ROS parameter values
type ParameterValue = boolean | number | string

interface Parameter {
  name: string
  value: ParameterValue
}

/**
 * Convert a TypeScript value to ROS2 ParameterValue structure
 * @param value The value to convert
 * @returns ROS2 ParameterValue object
 */
const toParameterValue = (value: ParameterValue): any => {
  if (typeof value === 'boolean') {
    return {
      type: 1, // PARAMETER_BOOL
      bool_value: value
    }
  } else if (typeof value === 'number') {
    // Check if it's an integer or float
    if (Number.isInteger(value)) {
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
const setParametersService = new ROSLIB.Service({
  ros: ros,
  name: '/session_configurator/set_parameters',
  serviceType: 'rcl_interfaces/SetParameters',
})

/**
 * Set a single ROS parameter on the session_configurator node
 * @param name Parameter name (e.g., 'decider.module')
 * @param value Parameter value (boolean, number, or string)
 * @param callback Callback function called on success
 */
export const setParameterRos = (name: string, value: ParameterValue, callback: () => void) => {
  const parameter = {
    name: name,
    value: toParameterValue(value)
  }

  const request = new ROSLIB.ServiceRequest({
    parameters: [parameter]
  }) as any

  setParametersService.callService(
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
 * Set multiple ROS parameters on the session_configurator node
 * @param parameters Array of parameter objects with name and value
 * @param callback Callback function called on success
 */
export const setParametersRos = (parameters: Parameter[], callback: () => void) => {
  const formattedParameters = parameters.map(param => ({
    name: param.name,
    value: toParameterValue(param.value)
  }))

  const request = new ROSLIB.ServiceRequest({
    parameters: formattedParameters
  }) as any

  setParametersService.callService(
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
const getParametersService = new ROSLIB.Service({
  ros: ros,
  name: '/session_configurator/get_parameters',
  serviceType: 'rcl_interfaces/GetParameters',
})

/**
 * Get current ROS parameters from the session_configurator node
 * @param names Array of parameter names to fetch
 * @returns Promise resolving to parameter values
 */
export const getParametersRos = (names: string[]): Promise<Map<string, boolean | number | string>> => {
  return new Promise((resolve, reject) => {
    const request = new ROSLIB.ServiceRequest({
      names: names
    }) as any

    getParametersService.callService(
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
