import React from 'react'
import ROSLIB from 'roslib'

import { ROS_URL } from '../utils/constants'
import { PositionMessage, StateMessage } from '../types/target'
import { EegBatchMessage, SampleMessage, EegTriggerMessage } from '../types/eeg'

export const ros = new ROSLIB.Ros({
  url: ROS_URL,
})

ros.on('connection', () => {
  console.log('ROS connected to', ROS_URL)
})

ros.on('error', (error) => {
  console.log('Error connecting ROS to', ROS_URL, ',error:', error)
})

ros.on('close', () => {
  console.log('ROS closed ws connection')
})

export const coilPositionSubscriber = new ROSLIB.Topic<PositionMessage>({
  ros: ros,
  name: '/neuronavigation/focus',
  messageType: 'neuronavigation_interfaces/PoseUsingEulerAngles',
})

/* Get maximum intensity service */
const getMaximumIntensityService = new ROSLIB.Service({
  ros: ros,
  name: '/targeting/get_maximum_intensity',
  serviceType: 'targeting_interfaces/GetMaximumIntensity',
})

export const getMaximumIntensity = (
  x: number,
  y: number,
  angle: number,
  algorithm: number,
  callback: (maximum_intensity: number) => void,
) => {
  const request = new ROSLIB.ServiceRequest({
    displacement_x: x,
    displacement_y: y,
    rotation_angle: angle,
    algorithm: {
      value: algorithm,
    },
  }) as any

  getMaximumIntensityService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to get maximum intensity')
      } else {
        callback(response.maximum_intensity)
      }
    },
    (error) => {
      console.log('ERROR: Failed to get maximum intensity, error:')
      console.log(error)
    },
  )
}

/* Toggle navigation service */
const toggleNavigationService = new ROSLIB.Service({
  ros: ros,
  name: '/planner/toggle_navigation',
  serviceType: 'ui_interfaces/ToggleNavigation',
})

const clearStateService = new ROSLIB.Service({
  ros: ros,
  name: '/planner/clear_state',
  serviceType: 'ui_interfaces/ClearState',
})

/* Listener for coil at target */
const coilAtTargetListener = new ROSLIB.Topic({
  ros: ros,
  name: '/neuronavigation/coil_at_target',
  messageType: 'std_msgs/Bool',
})
//coilAtTargetListener.subscribe(updateCoilAtTarget);

/* Listener for planner state */
export const plannerStateSubscriber = new ROSLIB.Topic<StateMessage>({
  ros: ros,
  name: '/planner/state',
  messageType: 'ui_interfaces/PlannerState',
})

/* Listener for eeg data */
export const eegDataSubscriber = new ROSLIB.Topic<EegBatchMessage>({
  ros: ros,
  name: '/eeg/batch_data',
  messageType: 'ui_interfaces/EegBatch',
})

/* Listener for node messages */
export const nodeMessageSubscriber = new ROSLIB.Topic({
  ros: ros,
  name: '/node/message',
  messageType: 'std_msgs/String',
})

/* Listener for planner state */
export const triggerSubscriber = new ROSLIB.Topic<EegTriggerMessage>({
  ros: ros,
  name: '/eeg/trigger',
  messageType: 'ui_interfaces/Trigger',
})

export const clearRosState = () => {
  const request = new ROSLIB.ServiceRequest({})
  console.log('clearing ros state')
  clearStateService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to clear state')
      }
    },
    (error) => {
      console.log('ERROR: Failed to clear state, error:')
      console.error(error)
    },
  )
}

/* Count valid trials service */
const countValidTrialsService = new ROSLIB.Service({
  ros: ros,
  name: '/experiment/count_valid_trials',
  serviceType: 'experiment_interfaces/CountValidTrials',
})

export const countValidTrials = (trials: any, callback: (numOfValidTrials: number) => void) => {
  const request = new ROSLIB.ServiceRequest({
    trials: trials,
  }) as any

  countValidTrialsService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to count valid trials: success field was false.')
      } else {
        callback(response.num_of_valid_trials)
      }
    },
    (error) => {
      console.log('ERROR: Failed to count valid trials, error:')
      console.log(error)
    },
  )
}

/* Perform experiment action

   TODO: After ROSLIB types are updated with the action support, bypassing
     the type checks (that is, ROSLIB as any) can be removed everywhere. */
const performExperimentActionClient: any = new (ROSLIB as any).Action({
  ros: ros,
  name: '/experiment/perform',
  actionType: 'experiment_interfaces/PerformExperiment',
})

export const performExperiment = (
  experiment: any,
  done_callback: (trialResults: any, success: boolean) => void,
  feedback_callback: (response: any) => void,
) => {
  console.log('simo')
  const goal: any = new (ROSLIB as any).ActionGoal({
    experiment: experiment,
  })
  console.log('esko')

  performExperimentActionClient.sendGoal(
    goal,
    (response: any) => {
      if (!response.success) {
        console.log('ERROR: Failed to perform experiment: success field was false.')
        done_callback(response.trial_results, false)
      } else {
        done_callback(response.trial_results, true)
      }
    },
    (feedback: any) => {
      feedback_callback(feedback)
    },
  )
}

/* Visualize targets service */
const visualizeTargetsService = new ROSLIB.Service({
  ros: ros,
  name: '/neuronavigation/visualize/targets',
  serviceType: 'neuronavigation_interfaces/VisualizeTargets',
})

export const visualizeTargets = (targets: any, is_ordered: boolean, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({}) as any

  request.targets = targets
  request.is_ordered = is_ordered

  visualizeTargetsService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to visualize targets: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to visualize targets, error:')
      console.log(error)
    },
  )
}

/* Pause experiment service */
const pauseExperimentService = new ROSLIB.Service({
  ros: ros,
  name: '/experiment/pause',
  serviceType: 'experiment_interfaces/PauseExperiment',
})

export const pauseExperiment = (callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({}) as any

  pauseExperimentService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to pause experiment: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to pause experiment, error:')
      console.log(error)
    },
  )
}

/* Resume experiment service */
const resumeExperimentService = new ROSLIB.Service({
  ros: ros,
  name: '/experiment/resume',
  serviceType: 'experiment_interfaces/ResumeExperiment',
})

export const resumeExperiment = (callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({}) as any

  resumeExperimentService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to resume experiment: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to resume experiment, error:')
      console.log(error)
    },
  )
}

/* Cancel experiment service */
const cancelExperimentService = new ROSLIB.Service({
  ros: ros,
  name: '/experiment/cancel',
  serviceType: 'experiment_interfaces/CancelExperiment',
})

export const cancelExperiment = (callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({}) as any

  cancelExperimentService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to cancel experiment: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to cancel experiment, error:')
      console.log(error)
    },
  )
}

/* List projects service */
const listProjectsService = new ROSLIB.Service({
  ros: ros,
  name: '/projects/list',
  serviceType: 'project_interfaces/ListProjects',
})

export const listProjects = (callback: (projects: string[]) => void) => {
  const request = new ROSLIB.ServiceRequest({}) as any

  listProjectsService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to list projects: success field was false.')
      } else {
        callback(response.projects)
      }
    },
    (error) => {
      console.log('ERROR: Failed to list projects, error:')
      console.log(error)
    },
  )
}

/* Set active project service */
const setActiveProjectService = new ROSLIB.Service({
  ros: ros,
  name: '/projects/active/set',
  serviceType: 'project_interfaces/SetActiveProject',
})

export const setActiveProject = (project: string, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    project: project,
  }) as any

  setActiveProjectService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to set active project: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to set active project, error:')
      console.log(error)
    },
  )
}

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
    },
  )
}

/* Set preprocessor enabled service */
const setPreprocessorEnabledService = new ROSLIB.Service({
  ros: ros,
  name: '/pipeline/preprocessor/enabled/set',
  serviceType: 'project_interfaces/SetPreprocessorEnabled',
})

export const setPreprocessorEnabledRos = (enabled: boolean, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    enabled: enabled,
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
    },
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
    },
  )
}

/* Set decider enabled service */
const setDeciderEnabledService = new ROSLIB.Service({
  ros: ros,
  name: '/pipeline/decider/enabled/set',
  serviceType: 'project_interfaces/SetDeciderEnabled',
})

export const setDeciderEnabledRos = (enabled: boolean, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    enabled: enabled,
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
    },
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
    },
  )
}

/* Set presenter enabled service */
const setPresenterEnabledService = new ROSLIB.Service({
  ros: ros,
  name: '/pipeline/presenter/enabled/set',
  serviceType: 'project_interfaces/SetPresenterEnabled',
})

export const setPresenterEnabledRos = (enabled: boolean, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    enabled: enabled,
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
    },
  )
}

/* Set dataset service */
const setDatasetService = new ROSLIB.Service({
  ros: ros,
  name: '/eeg_simulator/dataset/set',
  serviceType: 'project_interfaces/SetDataset',
})

export const setDatasetRos = (filename: string, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    filename: filename,
  }) as any

  setDatasetService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to set dataset: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to set dataset, error:')
      console.log(error)
    },
  )
}

/* Set playback service */
const setPlaybackService = new ROSLIB.Service({
  ros: ros,
  name: '/eeg_simulator/playback/set',
  serviceType: 'project_interfaces/SetPlayback',
})

export const setPlaybackRos = (playback: boolean, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    playback: playback,
  }) as any

  setPlaybackService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to set playback: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to set playback, error:')
      console.log(error)
    },
  )
}

/* Set loop service */
const setLoopService = new ROSLIB.Service({
  ros: ros,
  name: '/eeg_simulator/loop/set',
  serviceType: 'project_interfaces/SetLoop',
})

export const setLoopRos = (loop: boolean, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    loop: loop,
  }) as any

  setLoopService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to set loop: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to set loop, error:')
      console.log(error)
    },
  )
}

/* Set record data service */
const setRecordDataService = new ROSLIB.Service({
  ros: ros,
  name: '/eeg_recorder/record_data/set',
  serviceType: 'project_interfaces/SetRecordData',
})

export const setRecordDataRos = (recordData: boolean, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    record_data: recordData,
  }) as any

  setRecordDataService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to set loop: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to set loop, error:')
      console.log(error)
    },
  )
}
