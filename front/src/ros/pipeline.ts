import ROSLIB from 'roslib'
import { ros } from './ros'

/* Set preprocessor module service */
const setPreprocessorModuleService = new ROSLIB.Service({
  ros: ros,
  name: '/pipeline/preprocessor/module/set',
  serviceType: 'project_interfaces/SetPreprocessorModule',
})

export const setPreprocessorModuleRos = (module: string, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    module: module,
  }) as any

  setPreprocessorModuleService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to set preprocessor: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to set preprocessor, error:')
      console.log(error)
    }
  )
}

/* Set preprocessor enabled service */
const setPreprocessorEnabledService = new ROSLIB.Service({
  ros: ros,
  name: '/pipeline/preprocessor/enabled/set',
  serviceType: 'std_srvs/SetBool',
})

export const setPreprocessorEnabledRos = (enabled: boolean, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    data: enabled,
  }) as any

  setPreprocessorEnabledService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to set preprocessor enabled: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to set preprocessor enabled, error:')
      console.log(error)
    }
  )
}

/* Set decider module service */
const setDeciderModuleService = new ROSLIB.Service({
  ros: ros,
  name: '/pipeline/decider/module/set',
  serviceType: 'project_interfaces/SetDeciderModule',
})

export const setDeciderModuleRos = (module: string, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    module: module,
  }) as any

  setDeciderModuleService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to set decider: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to set decider, error:')
      console.log(error)
    }
  )
}

/* Set decider enabled service */
const setDeciderEnabledService = new ROSLIB.Service({
  ros: ros,
  name: '/pipeline/decider/enabled/set',
  serviceType: 'std_srvs/SetBool',
})

export const setDeciderEnabledRos = (enabled: boolean, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    data: enabled,
  }) as any

  setDeciderEnabledService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to set decider enabled: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to set decider enabled, error:')
      console.log(error)
    }
  )
}

/* Set presenter module service */
const setPresenterModuleService = new ROSLIB.Service({
  ros: ros,
  name: '/pipeline/presenter/module/set',
  serviceType: 'project_interfaces/SetPresenterModule',
})

export const setPresenterModuleRos = (module: string, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    module: module,
  }) as any

  setPresenterModuleService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to set presenter: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to set presenter, error:')
      console.log(error)
    }
  )
}

/* Set presenter enabled service */
const setPresenterEnabledService = new ROSLIB.Service({
  ros: ros,
  name: '/pipeline/presenter/enabled/set',
  serviceType: 'std_srvs/SetBool',
})

export const setPresenterEnabledRos = (enabled: boolean, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    data: enabled,
  }) as any

  setPresenterEnabledService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to set presenter enabled: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to set presenter enabled, error:')
      console.log(error)
    }
  )
}
