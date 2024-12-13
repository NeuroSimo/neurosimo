import React, { useContext, useEffect, useState, useRef } from 'react'
import styled from 'styled-components'

import { TabBar, Select, GrayedOutPanel, ActiveProps } from 'styles/General'

import { LocationSelector, Point } from 'components/Experiment/LocationSelector'
import { AngleSelector } from 'components/Experiment/AngleSelector'
import { IntensitySelector } from 'components/Experiment/IntensitySelector'
import { ToggleSwitch } from 'components/Experiment/ToggleSwitch'

import { ValidatedInput } from 'components/ValidatedInput'

import { SmallerTitle } from 'styles/ExperimentStyles'
import {
  StyledPanel,
  StyledButton,
  StyledRedButton,
  ProjectRow,
  ConfigRow,
  CloseConfigRow,
  ConfigLabel,
  IndentedLabel,
} from 'styles/General'

import {
  getMaximumIntensity,
  countValidTrials,
  listProjects,
  performExperiment,
  pauseExperiment,
  resumeExperiment,
  cancelExperiment,
  setActiveProject,
  visualizeTargets,
} from 'ros/ros'

import { SystemContext } from 'providers/SystemProvider'
import { ProjectContext } from 'providers/ProjectProvider'
import { HealthcheckContext, HealthcheckStatus } from 'providers/HealthcheckProvider'
import { ConfigContext } from 'providers/ConfigProvider'

import { formatTime } from 'utils/utils'
import { set } from 'cypress/types/lodash'

/* Styles for inputs for experiment metadata (= experiment and subject name) */
const ExperimentMetadata = styled.div`
  margin-bottom: 40px;
`

const InputRow = styled.div`
  display: flex;
  justify-content: flex-start;
  align-items: center;
  gap: 10px;
  margin-bottom: 10px;
`

const Label = styled.label`
  width: 150px;
  text-align: left;
  margin-right: 10px;
  margin-left: 30px;
  display: inline-block;
`

const Input = styled.input`
  width: 300px;
  padding: 8px;
  border: 1px solid #ccc;
  border-radius: 4px;
  outline: none;
  transition: background-color 0.2s;

  &:focus {
    background-color: transparent;
  }
`

const ExperimentTypeText = styled.div<ActiveProps>`
  cursor: pointer;
  display: inline-block;
  margin-right: 15px;
  margin-bottom: 10px;
  font-weight: ${(props) => (props.isActive ? 'bold' : 'normal')};
`

/* Styles for tab for pulse selection */
const PulseTab = styled.a<ActiveProps>`
  padding: 5px 11px;
  cursor: pointer;
  border: ${(props) => (props.isActive ? '1px solid #888' : '1px solid #ddd')};
  border-radius: 4px;
  background-color: ${(props) => (props.isActive ? 'white' : '#f5f5f5')};
  color: ${(props) => (props.isActive ? 'black' : '#333')};
  text-decoration: none;
  margin-right: 5px;
  font-weight: ${(props) => (props.isActive ? 'bold' : 'normal')};
  font-size: 18px;

  &:hover {
    background-color: #ddd;
  }
`

const PulseSelectorContainer = styled.div<ActiveProps>`
  display: flex;
  flex-direction: row;
  margin-bottom: 10px;
  margin-top: 10px;
  opacity: 0;
  visibility: hidden;
  transition: opacity 0.2s, visibility 0.2s;

  ${(props) =>
    props.isActive &&
    `
    opacity: 1;
    visibility: visible;
  `}
`

const StimulationParametersPanel = styled.div`
  display: grid;
  grid-template-rows: repeat(1, 1fr);
  grid-template-columns: repeat(4, 1fr);
  gap: 20px;
  width: 600px;
  margin-bottom: 20px;
`

const LocationPanel = styled(StyledPanel)`
  grid-row: 1 / 2;
  grid-column: 1 / 2;
  width: 626px;
  height: 500px;
`

const AnglePanel = styled(StyledPanel)`
  grid-row: 1 / 2;
  grid-column: 2 / 3;
  gap: 1.5rem;
  width: 597px;
  height: 500px;
`

const IntensityPanel = styled(StyledPanel)`
  grid-row: 1 / 2;
  grid-column: 3 / 4;
  gap: 1.5rem;
  width: 105px;
  height: 500px;
`

/* Timing (for paired pulses) */
const TimingPanel = styled(StyledPanel)<ActiveProps>`
  grid-row: 1 / 2;
  grid-column: 4 / 5;
  gap: 1.5rem;
  width: 270px;
  height: 100px;

  opacity: 0;
  visibility: hidden;
  transition: opacity 0.2s, visibility 0.2s;

  ${(props) =>
    props.isActive &&
    `
    opacity: 1;
    visibility: visible;
  `}
`

const TimingRow = styled.div`
  display: flex;
  justify-content: flex-start;
  align-items: center;
  margin-bottom: 10px;
  width: 350px;
`

const TimingLabel = styled.div`
  font-size: 16px;

  /* XXX: The negative margin is to align the text with the input field; the larger than normal
     arrow otherwise causes the text to be misaligned. */
  margin-top: -15px;

  margin-right: 40px;
  .arrow {
    font-size: 250%;
  }
`

/* Config panels */
const ConfigPanel = styled.div`
  display: grid;
  grid-template-rows: repeat(1, 1fr);
  grid-template-columns: repeat(6, 1fr);
  width: 600px;
  height: 550px;
  gap: 20px;
`

const TriggerPanel = styled(StyledPanel)`
  grid-row: 1 / 2;
  grid-column: 1 / 2;
  width: 300px;
  height: 240px;
`

const MepPanel = styled(StyledPanel)`
  grid-row: 1 / 2;
  grid-column: 2 / 3;
  width: 272px;
  height: 240px;
`

const TrialsPanel = styled(StyledPanel)`
  grid-row: 1 / 2;
  grid-column: 3 / 4;
  width: 270px;
  height: 240px;
`

const PausePanel = styled(StyledPanel)`
  grid-row: 1 / 2;
  grid-column: 4 / 5;
  width: 270px;
  height: 240px;
`

const ExperimentPanel = styled(StyledPanel)`
  grid-row: 1 / 2;
  grid-column: 5 / 6;
  width: 240px;
  height: 240px;
`

const StatusPanel = styled(StyledPanel)`
  grid-row: 1 / 2;
  grid-column: 6 / 7;
  width: 240px;
  height: 300px;
`

/* Triggers */
const TriggerRow = styled.div`
  display: flex;
  justify-content: flex-start;
  align-items: center;
  gap: 0px;
  margin-bottom: 10px;
`

const TriggerLabel = styled.label`
  width: 0px;
  font-size: 14px;
  font-weight: bold;
  text-align: left;
  display: inline-block;
`

const DelayLabel = styled.label`
  margin-left: -8px;
  margin-right: 15px;
  font-size: 14px;
  font-weight: bold;
  text-align: right;
  color: 'black';
`

/* TODO: Move type definitions elsewhere. */

type TriggerConfig = {
  enabled: boolean
  delay: number
}

type IntertrialInterval = {
  min: number
  max: number
  tolerance: number
}

type Metadata = {
  experiment_name: string
  subject_name: string
}

type Experiment = {
  metadata: Metadata

  trials: Trial[]
  intertrial_interval: IntertrialInterval

  randomize_trials: boolean
  wait_for_pedal_press: boolean

  autopause: boolean
  autopause_interval: number
}

type Target = {
  displacement_x: number
  displacement_y: number
  rotation_angle: number
  intensity: number
  algorithm: {
    value: number
  }
}

type TimeWindow = {
  start: number
  end: number
}

type PreactivationCheck = {
  enabled: boolean
  time_window: TimeWindow
  voltage_range_limit: number
}

type MepConfiguration = {
  emg_channel: number
  time_window: TimeWindow
  preactivation_check: PreactivationCheck
}

type Trial = {
  targets: Target[]
  pulse_times_since_trial_start: number[]

  triggers: TriggerConfig[]

  analyze_mep: boolean
  mep_config: MepConfiguration
}

enum StartButtonState {
  Start,
  Updating,
  Pausing,
  Pause,
  Resume,
}

enum CancelButtonState {
  Cancel,
  Canceling,
}

enum ExperimentState {
  NotRunning,
  Running,
  Paused,
  Canceled,
}

enum ExperimentTab {
  SingleLocation,
  MultipleLocations,
  PairedPulse,
}

/* Session storage utilities. */

const getData = (): any => {
  const data = sessionStorage.getItem('experiment')
  return data ? JSON.parse(data) : {}
}

const storeKey = (key: string, value: any) => {
  const currentData = getData()
  currentData[key] = value
  sessionStorage.setItem('experiment', JSON.stringify(currentData))
}

const getKey = (key: string, defaultValue: any): any => {
  const data = getData()
  return key in data ? data[key] : defaultValue
}

export const ExperimentView = () => {
  const { activeProject } = useContext(ProjectContext)

  const { mepHealthcheck } = useContext(HealthcheckContext)
  const [mepHealthcheckOk, setMepHealthcheckOk] = useState(false)

  const { session } = useContext(SystemContext)
  const { targetingAlgorithm } = useContext(ConfigContext)

  const [projects, setProjects] = useState<string[]>([])

  const [experimentName, setExperimentName] = useState<string>(() => getKey('experimentName', ''))
  const [subjectName, setSubjectName] = useState<string>(() => getKey('subjectName', ''))

  const [activeTab, setActiveTab] = useState<ExperimentTab>(() => getKey('activeTab', ExperimentTab.SingleLocation))

  /* For paired pulse experiments. */
  const [selectedPulse, setSelectedPulse] = useState<number>(() => getKey('selectedPulse', 1))

  const [selectedPointFirstPulse, setSelectedPointFirstPulse] = useState<Point[]>(() =>
    getKey('selectedPointFirstPulse', [{ x: 0, y: 0 }]),
  )
  const [selectedPointSecondPulse, setSelectedPointSecondPulse] = useState<Point[]>(() =>
    getKey('selectedPointSecondPulse', [{ x: 0, y: 0 }]),
  )

  const [selectedAngleFirstPulse, setSelectedAngleFirstPulse] = useState<number[]>(() =>
    getKey('selectedAngleFirstPulse', [0]),
  )
  const [selectedAngleSecondPulse, setSelectedAngleSecondPulse] = useState<number[]>(() =>
    getKey('selectedAngleSecondPulse', [0]),
  )

  const [intensityFirstPulse, setIntensityFirstPulse] = useState<number>(() => getKey('intensityFirstPulse', 10))
  const [intensitySecondPulse, setIntensitySecondPulse] = useState<number>(() => getKey('intensitySecondPulse', 10))

  const [pairedPulseDelay, setPairedPulseDelay] = useState<number>(() => getKey('pairedPulseDelay', 5))

  /* For single-pulse experiments. */
  const [selectedAngles, setSelectedAngles] = useState<number[]>(() => getKey('selectedAngles', [0]))
  const [selectedPoints, setSelectedPoints] = useState<Point[]>(() => getKey('selectedPoints', [{ x: 0, y: 0 }]))

  const [highlightedPoints, setHighlightedPoints] = useState<Point[]>([])
  const [highlightedAngles, setHighlightedAngles] = useState<number[]>([])

  /* TODO: Currently, the initial value set here needs to match the initial value in IntensitySelector
    component - remove the dependency. */
  const [intensity, setIntensity] = useState<number>(() => getKey('intensity', 10))
  const [maximumIntensity, setMaximumIntensity] = useState<number>(() => getKey('maximumIntensity', 100))

  const [trigger1Enabled, setTrigger1Enabled] = useState<boolean>(() => getKey('trigger1Enabled', true))
  const [trigger1Delay, setTrigger1Delay] = useState<number>(() => getKey('trigger1Delay', 0))

  const [trigger2Enabled, setTrigger2Enabled] = useState<boolean>(() => getKey('trigger2Enabled', true))
  const [trigger2Delay, setTrigger2Delay] = useState<number>(() => getKey('trigger2Delay', 0))

  const [mepEnabled, setMepEnabled] = useState<boolean>(() => getKey('mepEnabled', true))
  const [emgChannel, setEmgChannel] = useState<number>(() => getKey('emgChannel', 1))

  const [numOfRepetitions, setNumOfRepetitions] = useState<number>(() => getKey('numOfRepetitions', 10))
  const [waitForPedalPress, setwaitForPedalPress] = useState<boolean>(() => getKey('waitForPedalPress', false))

  const [itiMin, setItiMin] = useState<number>(() => getKey('itiMin', 3.5))
  const [itiMax, setItiMax] = useState<number>(() => getKey('itiMax', 4.5))

  const [autopause, setAutopause] = useState<boolean>(() => getKey('autopause', true))
  const [autopauseIntervalMinutes, setAutopauseIntervalMinutes] = useState<number>(() =>
    getKey('autopauseIntervalMinutes', 10),
  )

  const [numOfValidTrials, setNumOfValidTrials] = useState<number | null>(null)
  const [isValidating, setIsValidating] = useState<boolean>(false)
  const [numOfTrials, setNumOfTrials] = useState<number>(10)
  const [duration, setDuration] = useState<number | null>(null)

  const debounceTimerRef = useRef<NodeJS.Timeout | null>(null)

  const [startButtonState, setStartButtonState] = useState(StartButtonState.Start)
  const [cancelButtonState, setCancelButtonState] = useState(CancelButtonState.Cancel)

  /* Time (in seconds) at which the experiment was paused; used for showing the pause duration for the user. */
  const [pauseTime, setPauseTime] = useState<number | null>(null)

  const [trialNumber, setTrialNumber] = useState<number | null>(null)
  const [attemptNumber, setAttemptNumber] = useState<number | null>(null)
  const [experimentState, setExperimentState] = useState<number>(ExperimentState.NotRunning)

  const visualizeFeedback = (feedback: any) => {
    /* TODO: Only visualize the first target for now. */
    const target = feedback.trial.targets[0]
    const x: number = target.displacement_x
    const y: number = target.displacement_y
    const angle: number = target.rotation_angle

    setHighlightedPoints([{ x: x, y: y }])
    setHighlightedAngles([angle])
  }

  const perform = () => {
    const experiment: Experiment = formExperiment()

    const done_callback = (trial_results: any, success: boolean) => {
      setTrialNumber(null)
      setAttemptNumber(null)
      setExperimentState(ExperimentState.NotRunning)
      setCancelButtonState(CancelButtonState.Cancel)
      setHighlightedPoints([])
      setHighlightedAngles([])
    }

    const feedback_callback = (feedback: any) => {
      visualizeFeedback(feedback)
      setTrialNumber(feedback.trial_number)
      setAttemptNumber(feedback.attempt_number)
      setExperimentState(feedback.experiment_state.value)
    }
    performExperiment(experiment, done_callback, feedback_callback)
  }

  const handleProjectChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const newActiveProject = event.target.value
    setActiveProject(newActiveProject, () => {
      console.log('Active project set to ' + newActiveProject)
    })
  }

  const handleIntensityChange = (intensity: number) => {
    setIntensity(intensity)
  }

  const handleIntensityChangeForPairedPulse = (intensity: number) => {
    if (selectedPulse === 1) {
      setIntensityFirstPulse(intensity)
    } else {
      setIntensitySecondPulse(intensity)
    }
  }

  const formTrials = (): Trial[] => {
    let trials: Trial[] = []

    const triggers: TriggerConfig[] = [
      {
        enabled: trigger1Enabled,
        /* Delay is presented as milliseconds in the UI, transform to seconds. */
        delay: trigger1Delay / 1000,
      },
      {
        enabled: trigger2Enabled,
        /* Delay is presented as milliseconds in the UI, transform to seconds. */
        delay: trigger2Delay / 1000,
      },
    ]

    /* TODO: Hard-coded for now - make configurable. */
    const mep_config_time_window: TimeWindow = {
      start: 0.01,
      end: 0.04,
    }

    /* TODO: Hard-coded for now - make configurable. */
    const preactivation_check_time_window: TimeWindow = {
      start: -0.1,
      end: -0.01,
    }

    /* TODO: Hard-coded for now - make configurable. */
    const preactivation_check: PreactivationCheck = {
      enabled: false,
      time_window: preactivation_check_time_window,
      voltage_range_limit: 90,
    }

    const mep_config: MepConfiguration = {
      /* 0-based indexing is internally used for EMG channels, hence decrement to allow
        the user to use 1-based indexing. */
      emg_channel: emgChannel - 1,
      time_window: mep_config_time_window,
      preactivation_check: preactivation_check,
    }

    if (activeTab === ExperimentTab.MultipleLocations) {
      const singleRepetitionTrials: Trial[] = []

      selectedPoints.forEach((point: Point) => {
        selectedAngles.forEach((angle: number) => {
          const target: Target = {
            displacement_x: point.x,
            displacement_y: point.y,
            rotation_angle: angle,
            intensity: intensity,
            algorithm: {
              value: targetingAlgorithm,
            },
          }

          const trial: Trial = {
            targets: [target],
            pulse_times_since_trial_start: [0],

            triggers: triggers,

            /* Override mepEnabled if MEP healthcheck is not ok. */
            analyze_mep: mepHealthcheckOk ? mepEnabled : false,
            mep_config: mep_config,
          }
          singleRepetitionTrials.push(trial)
        })
      })
      trials = ([] as Trial[]).concat(...Array(numOfRepetitions).fill(singleRepetitionTrials))
    }
    if (activeTab === ExperimentTab.SingleLocation) {
      const target: Target = {
        displacement_x: selectedPoints[0].x,
        displacement_y: selectedPoints[0].y,
        rotation_angle: selectedAngles[0],
        intensity: intensity,
        algorithm: {
          value: targetingAlgorithm,
        },
      }

      const trial: Trial = {
        targets: [target],
        pulse_times_since_trial_start: [0],

        triggers: triggers,

        /* Override mepEnabled if MEP healthcheck is not ok. */
        analyze_mep: mepHealthcheckOk ? mepEnabled : false,
        mep_config: mep_config,
      }
      for (let i = 0; i < numOfTrials; i++) {
        trials.push(trial)
      }
    }
    if (activeTab === ExperimentTab.PairedPulse) {
      const targetFirstPulse: Target = {
        displacement_x: selectedPointFirstPulse[0].x,
        displacement_y: selectedPointFirstPulse[0].y,
        rotation_angle: selectedAngleFirstPulse[0],
        intensity: intensityFirstPulse,
        algorithm: {
          value: targetingAlgorithm,
        },
      }
      const targetSecondPulse: Target = {
        displacement_x: selectedPointSecondPulse[0].x,
        displacement_y: selectedPointSecondPulse[0].y,
        rotation_angle: selectedAngleSecondPulse[0],
        intensity: intensitySecondPulse,
        algorithm: {
          value: targetingAlgorithm,
        },
      }

      const trial: Trial = {
        targets: [targetFirstPulse, targetSecondPulse],
        pulse_times_since_trial_start: [0, pairedPulseDelay / 1000],

        triggers: triggers,

        /* Override mepEnabled if MEP healthcheck is not ok. */
        analyze_mep: mepHealthcheckOk ? mepEnabled : false,
        mep_config: mep_config,
      }
      for (let i = 0; i < numOfTrials; i++) {
        trials.push(trial)
      }
    }
    return trials
  }

  const formExperiment = (): Experiment => {
    const trials = formTrials()

    const experiment: Experiment = {
      metadata: {
        experiment_name: experimentName,
        subject_name: subjectName,
      },

      trials: trials,
      intertrial_interval: {
        min: itiMin,
        max: itiMax,
        tolerance: 0.0,
      },

      randomize_trials: true,
      wait_for_pedal_press: waitForPedalPress,

      autopause: autopause,
      autopause_interval: autopauseIntervalMinutes * 60,
    }
    return experiment
  }

  const callCountRef = useRef(0)

  const updateValidTrials = (experiment: Experiment) => {
    callCountRef.current += 1
    const currentCallCount = callCountRef.current

    setIsValidating(true)

    countValidTrials(experiment.trials, (numOfValidTrials) => {
      if (currentCallCount === callCountRef.current) {
        setNumOfValidTrials(numOfValidTrials)
        setStartButtonState(StartButtonState.Start)

        setIsValidating(false)
      }
    })
  }

  const updateValidTrialsWithDebounce = (experiment: Experiment) => {
    setStartButtonState(StartButtonState.Updating)

    if (debounceTimerRef.current) {
      clearTimeout(debounceTimerRef.current)
    }

    const timer = setTimeout(() => {
      updateValidTrials(experiment)
    }, 500)

    debounceTimerRef.current = timer

    // Cleanup function to clear the timer if the component is unmounted or if the effect runs again
    return () => {
      clearTimeout(timer)
    }
  }

  const changeActiveTab = (activeTab: ExperimentTab) => {
    setActiveTab(activeTab)

    switch (activeTab) {
      case ExperimentTab.SingleLocation:
        setSelectedAngles([0])
        setSelectedPoints([{ x: 0, y: 0 }])

        setNumOfTrials(10)
        setNumOfValidTrials(null)

        setNumOfRepetitions(1)

        break

      case ExperimentTab.MultipleLocations:
        setNumOfRepetitions(3)

        break

      case ExperimentTab.PairedPulse:
        setNumOfTrials(10)
        setNumOfValidTrials(null)
        setDuration(null)

        setNumOfRepetitions(1)

        break
    }
  }

  const validateTrials = () => {
    const experiment: Experiment = formExperiment()
    updateValidTrials(experiment)
  }

  /* Set list of projects. */
  useEffect(() => {
    listProjects((projects) => {
      setProjects(projects)
    })
  }, [])

  /* Updates the maximum intensity display. */
  useEffect(() => {
    let x: number
    let y: number
    let angle: number
    if (activeTab === ExperimentTab.PairedPulse) {
      x = selectedPulse === 1 ? selectedPointFirstPulse[0].x : selectedPointSecondPulse[0].x
      y = selectedPulse === 1 ? selectedPointFirstPulse[0].y : selectedPointSecondPulse[0].y
      angle = selectedPulse === 1 ? selectedAngleFirstPulse[0] : selectedAngleSecondPulse[0]
    } else {
      if (selectedPoints.length === 1 && selectedAngles.length === 1) {
        x = selectedPoints[0].x
        y = selectedPoints[0].y
        angle = selectedAngles[0]
      } else {
        return
      }
    }
    getMaximumIntensity(x, y, angle, targetingAlgorithm, (maximum_intensity) => {
      setMaximumIntensity(maximum_intensity)
    })
  }, [
    selectedAngles,
    selectedPoints,
    targetingAlgorithm,
    selectedAngleFirstPulse,
    selectedAngleSecondPulse,
    selectedPointFirstPulse,
    selectedPointSecondPulse,
    selectedPulse,
  ])

  /* Updates the target visualization in neuronavigation for single location or multiple locations (paired pulse
     is handled separately). */
  useEffect(() => {
    if (activeTab === ExperimentTab.PairedPulse) {
      return
    }

    let targets: any[] = []
    if (selectedPoints.length >= 1 && selectedAngles.length === 1) {
      targets = selectedPoints.map((point) => {
        return {
          displacement_x: point.x,
          displacement_y: point.y,
          rotation_angle: selectedAngles[0],
          intensity: intensity,
        }
      })
    }
    const is_ordered = true
    visualizeTargets(targets, is_ordered, () => {
      console.log('Visualization successful')
    })
  }, [selectedAngles, selectedPoints, intensity, activeTab])

  /* Updates the target visualization in neuronavigation for paired pulses. */
  useEffect(() => {
    if (activeTab !== ExperimentTab.PairedPulse) {
      return
    }

    let targets: any[] = []
    if (selectedPointFirstPulse.length === 1 && selectedAngleFirstPulse.length === 1) {
      targets = [
        {
          displacement_x: selectedPointFirstPulse[0].x,
          displacement_y: selectedPointFirstPulse[0].y,
          rotation_angle: selectedAngleFirstPulse[0],
          intensity: intensityFirstPulse,
        },
      ]
    }
    if (selectedPointSecondPulse.length === 1 && selectedAngleSecondPulse.length === 1) {
      targets = [
        ...targets,
        {
          displacement_x: selectedPointSecondPulse[0].x,
          displacement_y: selectedPointSecondPulse[0].y,
          rotation_angle: selectedAngleSecondPulse[0],
          intensity: intensitySecondPulse,
        },
      ]
    }
    const is_ordered = true
    visualizeTargets(targets, is_ordered, () => {
      console.log('Visualization of paired pulse successful')
    })
  }, [
    selectedPointFirstPulse,
    selectedPointSecondPulse,
    selectedAngleFirstPulse,
    selectedAngleSecondPulse,
    intensityFirstPulse,
    intensitySecondPulse,
    activeTab,
  ])

  /* Update the number of valid trials. */
  useEffect(() => {
    /* Do it automatically only for single location and multiple locations, not for paired pulse due to its slowness. */
    if (activeTab === ExperimentTab.PairedPulse) {
      return
    }
    if (selectedPoints.length == 0 || selectedAngles.length == 0) {
      setNumOfTrials(0)
      setNumOfValidTrials(null)
      setDuration(null)

      return
    }
    const experiment: Experiment = formExperiment()

    updateValidTrialsWithDebounce(experiment)
  }, [selectedAngles, selectedPoints, intensity, targetingAlgorithm, numOfRepetitions, numOfTrials])

  /* Update the total number of trials. */
  useEffect(() => {
    if (activeTab === ExperimentTab.MultipleLocations) {
      setNumOfTrials(numOfRepetitions * selectedPoints.length * selectedAngles.length)
    }
  }, [numOfRepetitions, selectedPoints, selectedAngles, activeTab])

  /* Do not show valid trials in paired pulse tab if the number of valid trials is not yet known. */
  useEffect(() => {
    if (activeTab === ExperimentTab.PairedPulse) {
      setNumOfValidTrials(null)
      setDuration(null)
    }
  }, [
    numOfTrials,
    selectedPointFirstPulse,
    selectedPointSecondPulse,
    selectedAngleFirstPulse,
    selectedAngleSecondPulse,
    intensityFirstPulse,
    intensitySecondPulse,
    targetingAlgorithm,
  ])

  /* Update the experiment duration. */
  useEffect(() => {
    if (numOfValidTrials === null) {
      return
    }
    const duration = (numOfValidTrials * (itiMin + itiMax)) / 2
    setDuration(duration)
  }, [itiMin, itiMax, numOfValidTrials])

  /* Update experiment state. */
  useEffect(() => {
    switch (experimentState) {
      case ExperimentState.NotRunning:
        setStartButtonState(StartButtonState.Start)
        break

      case ExperimentState.Running:
        setStartButtonState(StartButtonState.Pause)
        break

      case ExperimentState.Paused:
        setStartButtonState(StartButtonState.Resume)
        if (session) {
          setPauseTime(session.time)
        }
        break
    }
  }, [experimentState])

  /* Update MEP healthcheck ok status. */
  useEffect(() => {
    setMepHealthcheckOk(mepHealthcheck?.status.value === HealthcheckStatus.READY)
  }, [mepHealthcheck])

  /* Update session storage. */
  useEffect(() => {
    storeKey('experimentName', experimentName)
  }, [experimentName])

  useEffect(() => {
    storeKey('subjectName', subjectName)
  }, [subjectName])

  useEffect(() => {
    storeKey('activeTab', activeTab)
  }, [activeTab])

  useEffect(() => {
    storeKey('selectedPulse', selectedPulse)
  }, [selectedPulse])

  useEffect(() => {
    storeKey('selectedPointFirstPulse', selectedPointFirstPulse)
  }, [selectedPointFirstPulse])

  useEffect(() => {
    storeKey('selectedPointSecondPulse', selectedPointSecondPulse)
  }, [selectedPointSecondPulse])

  useEffect(() => {
    storeKey('selectedAngleFirstPulse', selectedAngleFirstPulse)
  }, [selectedAngleFirstPulse])

  useEffect(() => {
    storeKey('selectedAngleSecondPulse', selectedAngleSecondPulse)
  }, [selectedAngleSecondPulse])

  useEffect(() => {
    storeKey('pairedPulseDelay', pairedPulseDelay)
  }, [pairedPulseDelay])

  useEffect(() => {
    storeKey('selectedAngles', selectedAngles)
  }, [selectedAngles])

  useEffect(() => {
    storeKey('selectedPoints', selectedPoints)
  }, [selectedPoints])

  useEffect(() => {
    storeKey('intensity', intensity)
  }, [intensity])

  useEffect(() => {
    storeKey('intensityFirstPulse', intensityFirstPulse)
  }, [intensityFirstPulse])

  useEffect(() => {
    storeKey('intensitySecondPulse', intensitySecondPulse)
  }, [intensitySecondPulse])

  useEffect(() => {
    storeKey('maximumIntensity', maximumIntensity)
  }, [maximumIntensity])

  useEffect(() => {
    storeKey('trigger1Enabled', trigger1Enabled)
  }, [trigger1Enabled])

  useEffect(() => {
    storeKey('trigger1Delay', trigger1Delay)
  }, [trigger1Delay])

  useEffect(() => {
    storeKey('trigger2Enabled', trigger2Enabled)
  }, [trigger2Enabled])

  useEffect(() => {
    storeKey('trigger2Delay', trigger2Delay)
  }, [trigger2Delay])

  useEffect(() => {
    storeKey('mepEnabled', mepEnabled)
  }, [mepEnabled])

  useEffect(() => {
    storeKey('emgChannel', emgChannel)
  }, [emgChannel])

  useEffect(() => {
    storeKey('numOfRepetitions', numOfRepetitions)
  }, [numOfRepetitions])

  useEffect(() => {
    storeKey('waitForPedalPress', waitForPedalPress)
  }, [waitForPedalPress])

  useEffect(() => {
    storeKey('itiMin', itiMin)
  }, [itiMin])

  useEffect(() => {
    storeKey('itiMax', itiMax)
  }, [itiMax])

  useEffect(() => {
    storeKey('autopause', autopause)
  }, [autopause])

  useEffect(() => {
    storeKey('autopauseIntervalMinutes', autopauseIntervalMinutes)
  }, [autopauseIntervalMinutes])

  /* Utilities */
  const formatExperimentState = (): string => {
    switch (experimentState) {
      case ExperimentState.NotRunning:
        return 'Not running'
      case ExperimentState.Running:
        return 'Running'
      case ExperimentState.Paused:
        return 'Paused'
      case ExperimentState.Canceled:
        return 'Canceled'
    }
    return 'Unknown'
  }

  const formatTrialNumber = () => {
    if (trialNumber === null || numOfValidTrials === null) {
      return '\u2013'
    }
    return trialNumber.toString() + '/' + numOfValidTrials.toString()
  }

  const startButtonStateToString = (startButtonState: StartButtonState): string => {
    switch (startButtonState) {
      case StartButtonState.Updating:
        return 'Updating...'
      case StartButtonState.Start:
        return 'Start'
      case StartButtonState.Pausing:
        return 'Pausing...'
      case StartButtonState.Pause:
        return 'Pause'
      case StartButtonState.Resume:
        return 'Resume'
    }
  }

  const cancelButtonStateToString = (cancelButtonState: CancelButtonState): string => {
    switch (cancelButtonState) {
      case CancelButtonState.Cancel:
        return 'Cancel'
      case CancelButtonState.Canceling:
        return 'Canceling...'
    }
  }

  const runStartButtonAction = (startButtonState: StartButtonState) => {
    switch (startButtonState) {
      case StartButtonState.Updating:
        break
      case StartButtonState.Start:
        perform()
        break
      case StartButtonState.Pause:
        pauseExperiment(() => {
          setStartButtonState(StartButtonState.Pausing)
        })
        break
      case StartButtonState.Resume:
        resumeExperiment(() => {
          console.log('resumed')
        })
        break
    }
  }

  const runCancelButtonAction = () => {
    cancelExperiment(() => {
      setCancelButtonState(CancelButtonState.Canceling)
    })
  }

  return (
    <>
      <ExperimentMetadata>
        <ProjectRow>
          <Label>Project:</Label>
          <Select onChange={handleProjectChange} value={activeProject}>
            {projects.map((project, index) => (
              <option key={index} value={project}>
                {project}
              </option>
            ))}
          </Select>
        </ProjectRow>

        <InputRow>
          <Label>Experiment:</Label>
          <Input
            type='text'
            placeholder='E.g., Resting motor threshold experiment 1'
            value={experimentName}
            onChange={(e) => setExperimentName(e.target.value)}
            style={{ backgroundColor: experimentName ? 'transparent' : 'lightgray' }}
          />
        </InputRow>
        <InputRow>
          <Label>Subject:</Label>
          <Input
            type='text'
            placeholder='E.g., Subject 1'
            value={subjectName}
            onChange={(e) => setSubjectName(e.target.value)}
            style={{ backgroundColor: subjectName ? 'transparent' : 'lightgray' }}
          />
        </InputRow>
      </ExperimentMetadata>

      <TabBar>
        <ExperimentTypeText
          onClick={() => changeActiveTab(ExperimentTab.SingleLocation)}
          isActive={activeTab === ExperimentTab.SingleLocation}
        >
          Single Location
        </ExperimentTypeText>
        <ExperimentTypeText
          onClick={() => changeActiveTab(ExperimentTab.MultipleLocations)}
          isActive={activeTab === ExperimentTab.MultipleLocations}
        >
          Multiple Locations
        </ExperimentTypeText>
        <ExperimentTypeText
          onClick={() => changeActiveTab(ExperimentTab.PairedPulse)}
          isActive={activeTab === ExperimentTab.PairedPulse}
        >
          Paired Pulse
        </ExperimentTypeText>
      </TabBar>
      <PulseSelectorContainer isActive={activeTab === ExperimentTab.PairedPulse}>
        <PulseTab onClick={() => setSelectedPulse(1)} isActive={selectedPulse === 1}>
          1
        </PulseTab>
        <PulseTab onClick={() => setSelectedPulse(2)} isActive={selectedPulse === 2}>
          2
        </PulseTab>
      </PulseSelectorContainer>
      <StimulationParametersPanel>
        <LocationPanel>
          {activeTab === ExperimentTab.SingleLocation && (
            <LocationSelector
              selectedPoints={selectedPoints}
              setSelectedPoints={setSelectedPoints}
              highlightedPoints={highlightedPoints}
            />
          )}
          {activeTab === ExperimentTab.MultipleLocations && (
            <LocationSelector
              selectedPoints={selectedPoints}
              setSelectedPoints={setSelectedPoints}
              highlightedPoints={highlightedPoints}
              multiSelectMode={true}
            />
          )}
          {activeTab === ExperimentTab.PairedPulse && (
            <LocationSelector
              selectedPoints={selectedPulse === 1 ? selectedPointFirstPulse : selectedPointSecondPulse}
              setSelectedPoints={selectedPulse === 1 ? setSelectedPointFirstPulse : setSelectedPointSecondPulse}
              highlightedPoints={highlightedPoints}
            />
          )}
        </LocationPanel>
        <AnglePanel>
          {activeTab === ExperimentTab.SingleLocation && (
            <AngleSelector
              selectedAngles={selectedAngles}
              setSelectedAngles={setSelectedAngles}
              highlightedAngles={highlightedAngles}
            />
          )}
          {activeTab === ExperimentTab.MultipleLocations && (
            <AngleSelector
              selectedAngles={selectedAngles}
              setSelectedAngles={setSelectedAngles}
              highlightedAngles={highlightedAngles}
              multiSelectMode={true}
            />
          )}
          {activeTab === ExperimentTab.PairedPulse && (
            <AngleSelector
              selectedAngles={selectedPulse === 1 ? selectedAngleFirstPulse : selectedAngleSecondPulse}
              setSelectedAngles={selectedPulse === 1 ? setSelectedAngleFirstPulse : setSelectedAngleSecondPulse}
              highlightedAngles={highlightedAngles}
            />
          )}
        </AnglePanel>
        <IntensityPanel>
          {activeTab === ExperimentTab.SingleLocation && (
            <IntensitySelector
              value={intensity}
              min={0}
              max={150}
              showMaximumIntensity={true}
              maximumIntensity={maximumIntensity}
              onValueChange={handleIntensityChange}
            />
          )}
          {activeTab === ExperimentTab.MultipleLocations && (
            <IntensitySelector
              value={intensity}
              min={0}
              max={150}
              showMaximumIntensity={false}
              maximumIntensity={0}
              onValueChange={handleIntensityChange}
            />
          )}
          {activeTab === ExperimentTab.PairedPulse && (
            <IntensitySelector
              value={selectedPulse === 1 ? intensityFirstPulse : intensitySecondPulse}
              min={0}
              max={150}
              showMaximumIntensity={true}
              maximumIntensity={maximumIntensity}
              onValueChange={handleIntensityChangeForPairedPulse}
            />
          )}
        </IntensityPanel>
        <TimingPanel isActive={activeTab === ExperimentTab.PairedPulse}>
          <SmallerTitle>Timing</SmallerTitle>
          <TimingRow>
            <TimingLabel>
              1 <span className='arrow'>&rarr;</span> 2
            </TimingLabel>
            <DelayLabel>Delay (ms)</DelayLabel>
            <ValidatedInput value={pairedPulseDelay} min={0} max={1000} onChange={setPairedPulseDelay} />
          </TimingRow>
        </TimingPanel>
      </StimulationParametersPanel>
      <ConfigPanel>
        <TriggerPanel>
          <SmallerTitle>Triggers out</SmallerTitle>
          <TriggerRow>
            <TriggerLabel>1</TriggerLabel>
            <ToggleSwitch type='flat' checked={trigger1Enabled} onChange={setTrigger1Enabled} />
            <GrayedOutPanel isGrayedOut={!trigger1Enabled}>
              <DelayLabel>Delay (ms)</DelayLabel>
            </GrayedOutPanel>
            <GrayedOutPanel isGrayedOut={!trigger1Enabled}>
              <ValidatedInput
                value={trigger1Delay}
                min={-99}
                max={99}
                onChange={setTrigger1Delay}
                disabled={!trigger1Enabled}
              />
            </GrayedOutPanel>
          </TriggerRow>
          <TriggerRow>
            <TriggerLabel>2</TriggerLabel>
            <ToggleSwitch type='flat' checked={trigger2Enabled} onChange={setTrigger2Enabled} />
            <GrayedOutPanel isGrayedOut={!trigger2Enabled}>
              <DelayLabel>Delay (ms)</DelayLabel>
            </GrayedOutPanel>
            <GrayedOutPanel isGrayedOut={!trigger2Enabled}>
              <ValidatedInput
                value={trigger2Delay}
                min={-99}
                max={99}
                onChange={setTrigger2Delay}
                disabled={!trigger2Enabled}
              />
            </GrayedOutPanel>
          </TriggerRow>
        </TriggerPanel>
        <MepPanel isGrayedOut={!mepHealthcheckOk}>
          <SmallerTitle>MEP analysis</SmallerTitle>
          <ConfigRow>
            <ConfigLabel>Enabled:</ConfigLabel>
            <ToggleSwitch type='flat' checked={mepEnabled} onChange={setMepEnabled} disabled={!mepHealthcheckOk} />
          </ConfigRow>
          <GrayedOutPanel isGrayedOut={!mepEnabled || !mepHealthcheckOk}>
            <ConfigRow>
              <IndentedLabel>EMG channel:</IndentedLabel>
              {/*
                  TODO: Validate EMG channel by passing a prop to ValidatedInput, based on the
                  number of available EMG channels, published in EegInfo topic by EEG bridge.
                */}
              <ValidatedInput
                type='text'
                value={emgChannel}
                min={1}
                max={16}
                onChange={setEmgChannel}
                disabled={!mepEnabled}
              />
            </ConfigRow>
          </GrayedOutPanel>
        </MepPanel>
        <TrialsPanel>
          <SmallerTitle>Trials</SmallerTitle>
          {activeTab === ExperimentTab.MultipleLocations && (
            <ConfigRow>
              <ConfigLabel># of repetitions:</ConfigLabel>
              <ValidatedInput type='text' value={numOfRepetitions} min={1} max={999} onChange={setNumOfRepetitions} />
            </ConfigRow>
          )}
          {activeTab !== ExperimentTab.MultipleLocations && (
            <ConfigRow>
              <ConfigLabel># of trials:</ConfigLabel>
              <ValidatedInput type='text' value={numOfTrials} min={1} max={999} onChange={setNumOfTrials} />
            </ConfigRow>
          )}
          <ConfigRow>
            <ConfigLabel>Wait for pedal press:</ConfigLabel>
            <ToggleSwitch type='flat' checked={waitForPedalPress} onChange={setwaitForPedalPress} />
          </ConfigRow>
          <GrayedOutPanel isGrayedOut={waitForPedalPress || numOfTrials === null || numOfTrials < 2}>
            <ConfigRow>
              <ConfigLabel>Interval (s):</ConfigLabel>
            </ConfigRow>
            <CloseConfigRow>
              <IndentedLabel>Min</IndentedLabel>
              <ValidatedInput
                type='number'
                value={itiMin}
                min={0.1}
                max={100}
                step={0.1}
                onChange={setItiMin}
                disabled={waitForPedalPress || numOfTrials === null || numOfTrials < 2}
              />
            </CloseConfigRow>
            <CloseConfigRow>
              <IndentedLabel>Max</IndentedLabel>
              <ValidatedInput
                type='number'
                value={itiMax}
                min={0.1}
                max={100}
                step={0.1}
                onChange={setItiMax}
                disabled={waitForPedalPress || numOfTrials === null || numOfTrials < 2}
              />
            </CloseConfigRow>
          </GrayedOutPanel>
        </TrialsPanel>
        <PausePanel>
          <SmallerTitle>Pause</SmallerTitle>
          <ConfigRow>
            <ConfigLabel>Automatic:</ConfigLabel>
            <ToggleSwitch type='flat' checked={autopause} onChange={setAutopause} />
          </ConfigRow>
          <GrayedOutPanel isGrayedOut={!autopause}>
            <ConfigRow>
              <IndentedLabel>Interval (min)</IndentedLabel>
              <ValidatedInput
                type='number'
                value={autopauseIntervalMinutes}
                min={1}
                max={99}
                onChange={setAutopauseIntervalMinutes}
                disabled={!autopause}
              />
            </ConfigRow>
          </GrayedOutPanel>
        </PausePanel>
        <ExperimentPanel>
          <SmallerTitle>Experiment</SmallerTitle>
          <ConfigRow>
            <ConfigLabel>Trials:</ConfigLabel>
          </ConfigRow>
          <CloseConfigRow>
            <IndentedLabel>Total</IndentedLabel>
            <ConfigLabel>{numOfTrials !== null ? numOfTrials : '\u2013'}</ConfigLabel>
          </CloseConfigRow>
          <CloseConfigRow>
            <IndentedLabel>Valid</IndentedLabel>
            <ConfigLabel>{numOfValidTrials !== null ? numOfValidTrials : '\u2013'}</ConfigLabel>
          </CloseConfigRow>
          <ConfigRow>
            <ConfigLabel>Experiment:</ConfigLabel>
          </ConfigRow>
          <CloseConfigRow>
            <IndentedLabel>Duration</IndentedLabel>
            <ConfigLabel>{duration ? formatTime(duration) : '\u2013'}</ConfigLabel>
          </CloseConfigRow>
          <CloseConfigRow></CloseConfigRow>
          <StyledButton isHidden={activeTab !== ExperimentTab.PairedPulse} onClick={() => validateTrials()}>
            {isValidating ? 'Validating...' : 'Validate'}
          </StyledButton>
        </ExperimentPanel>
        <StatusPanel>
          <SmallerTitle>Status</SmallerTitle>
          <ConfigRow>
            <ConfigLabel>Experiment:</ConfigLabel>
            <ConfigLabel>{formatExperimentState()} </ConfigLabel>
          </ConfigRow>
          <ConfigRow>
            <IndentedLabel>Trial</IndentedLabel>
            <ConfigLabel>{formatTrialNumber()} </ConfigLabel>
          </ConfigRow>
          <CloseConfigRow>
            <IndentedLabel>Attempt</IndentedLabel>
            <ConfigLabel>{attemptNumber !== null ? attemptNumber : '\u2013'}</ConfigLabel>
          </CloseConfigRow>
          <CloseConfigRow></CloseConfigRow>
          <ConfigRow>
            <ConfigLabel>Time:</ConfigLabel>
            <ConfigLabel>
              {experimentState === ExperimentState.Running || experimentState === ExperimentState.Paused
                ? formatTime(session?.time)
                : '\u2013'}
            </ConfigLabel>
          </ConfigRow>
          {/* Gray out 'Paused for' if experiment is not paused. */}
          <GrayedOutPanel isGrayedOut={!(experimentState === ExperimentState.Paused)}>
            <ConfigRow>
              <ConfigLabel>Paused for:</ConfigLabel>
              <ConfigLabel>
                {experimentState === ExperimentState.Paused && pauseTime && session
                  ? formatTime(session?.time - pauseTime)
                  : '\u2013'}
              </ConfigLabel>
            </ConfigRow>
          </GrayedOutPanel>
          <CloseConfigRow></CloseConfigRow>
          <StyledButton
            onClick={() => runStartButtonAction(startButtonState)}
            disabled={
              startButtonState === StartButtonState.Updating ||
              startButtonState === StartButtonState.Pausing ||
              selectedPoints.length == 0 ||
              selectedAngles.length == 0
            }
          >
            {startButtonStateToString(startButtonState)}
          </StyledButton>
          <StyledRedButton
            onClick={() => runCancelButtonAction()}
            disabled={
              experimentState === ExperimentState.NotRunning || cancelButtonState == CancelButtonState.Canceling
            }
          >
            {cancelButtonStateToString(cancelButtonState)}
          </StyledRedButton>
        </StatusPanel>
      </ConfigPanel>
    </>
  )
}
