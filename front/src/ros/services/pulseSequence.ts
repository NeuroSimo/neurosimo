import ROSLIB from 'roslib'

import { Pulse, PulseSequence } from 'types/pulseSequence'
import { isOfPulseSequenceChangeableKey, objectKeysToSnakeCase } from 'utils'
import { ros } from 'ros/ros'

/* Set up set_pulse_sequence_intensity service. */
const setPulseSequenceIntensityService = new ROSLIB.Service({
  ros: ros,
  name: '/planner/set_pulse_sequence_intensity',
  serviceType: 'ui_interfaces/SetPulseSequenceIntensity',
})
/* Set up set_pulse_sequence_isi service. */
const setPulseSequenceIsiService = new ROSLIB.Service({
  ros: ros,
  name: '/planner/set_pulse_sequence_isi',
  serviceType: 'ui_interfaces/SetPulseSequenceIsi',
})
/* Set up toggle_select service. */
const togglePulseSequenceSelectService = new ROSLIB.Service({
  ros: ros,
  name: '/planner/toggle_select_pulse_sequence',
  serviceType: 'ui_interfaces/ToggleSelectPulseSequence',
})

/* Set up add_target service. */
const addPulseSequenceService = new ROSLIB.Service({
  ros: ros,
  name: '/planner/add_pulse_sequence',
  serviceType: 'ui_interfaces/AddPulseSequence',
})

/* Set up remove_pulse_sequence service. */
const removePulseSequenceService = new ROSLIB.Service({
  ros: ros,
  name: '/planner/remove_pulse_sequence',
  serviceType: 'ui_interfaces/RemovePulseSequence',
})
/* Set up rename_pulse_sequence service. */
const renamePulseSequenceService = new ROSLIB.Service({
  ros: ros,
  name: '/planner/rename_pulse_sequence',
  serviceType: 'ui_interfaces/RenamePulseSequence',
})

const addPulseToPulseSequenceService = new ROSLIB.Service({
  ros: ros,
  name: '/planner/add_pulse_to_pulse_sequence',
  serviceType: 'ui_interfaces/AddPulseToPulseSequence',
})

export const pulseSequenceServicesByKey = {
  name: renamePulseSequenceService,
  selected: togglePulseSequenceSelectService,
  isi: setPulseSequenceIsiService,
  intensity: setPulseSequenceIntensityService,
}

export const addPulseSequenceToRos = (pulses: Pulse[]) => {
  const snakeCasePulses = objectKeysToSnakeCase({
    pulses,
  })

  const request = new ROSLIB.ServiceRequest({
    pulses: snakeCasePulses.pulses,
  })

  addPulseSequenceService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to add pulse sequence')
      }
    },
    (error) => {
      console.log('ERROR: Failed to add pulse sequence, error:')
      console.error(error)
    },
  )
}

export const addPulseToPulseSequenceInRos = (sequence: PulseSequence, pulse: Pulse) => {
  const snakeCasePulse = objectKeysToSnakeCase(pulse)

  const request = new ROSLIB.ServiceRequest({
    pulse: snakeCasePulse,
    name: sequence.name,
  })

  addPulseToPulseSequenceService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to add pulse sequence')
      }
    },
    (error) => {
      console.log('ERROR: Failed to add pulse sequence, error:')
      console.error(error)
    },
  )
}

export const updatePulseSequenceInRos = (sequence: PulseSequence, key: string, value: any, toggle: boolean) => {
  if (!isOfPulseSequenceChangeableKey(key)) {
    console.error(`Key ${key} is not changeable`)
    return
  }

  let requestObject = {
    name: sequence.name,
  }
  if (!toggle) {
    const requestKey = `new_${key}`
    requestObject = {
      ...requestObject,
      [requestKey]: value,
    }
  }
  const request = new ROSLIB.ServiceRequest(requestObject)

  pulseSequenceServicesByKey[key].callService(
    request,
    (result) => {
      if (!result.success) {
        console.error(`ERROR: Failed to change key '${key}' from ${sequence[key]} to ${value}`)
      } else {
        console.log(`Changed ${sequence.name} key '${key}' from ${sequence[key]} to ${value}`)
      }
    },
    (error) => {
      console.error(error)
    },
  )
}

export const removePulseSequenceInRos = (pulseSequence: PulseSequence) => {
  const request = new ROSLIB.ServiceRequest({
    name: pulseSequence.name,
  })
  removePulseSequenceService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to remove pulse sequence', pulseSequence)
      }
    },
    (error) => {
      console.log('ERROR: Failed to remove pulse sequence', pulseSequence, ', error:')
      console.error(error)
    },
  )
}
