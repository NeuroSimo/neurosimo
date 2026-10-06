import React, { useContext } from 'react'
import styled from 'styled-components'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'
import { faWindowRestore } from '@fortawesome/free-solid-svg-icons'

import {
  TelemetryTitle,
  TelemetryPanel,
  StateRow,
  StateTitle,
  IndentedStateTitle,
  StateValue,
  StyledButton,
  StyledRedButton,
  TELEMETRY_EXPERIMENT_RIGHT,
  TELEMETRY_EXPERIMENT_WIDTH,
  palette,
} from 'styles/General'
import { ExperimentContext } from 'providers/ExperimentProvider'
import { pauseExperimentRos, resumeExperimentRos } from 'ros/experiment'

const ExperimentStateTitle = styled(TelemetryTitle)`
  width: ${TELEMETRY_EXPERIMENT_WIDTH}px;
  right: ${TELEMETRY_EXPERIMENT_RIGHT}px;
  padding-right: 6px;
`

const InfoIcon = styled.button<{ disabled: boolean }>`
  background: none;
  border: none;
  border-radius: 3px;
  width: 22px;
  height: 22px;
  cursor: ${props => props.disabled ? 'not-allowed' : 'pointer'};
  display: flex;
  align-items: center;
  justify-content: center;
  color: ${palette.blue};
  font-size: 12px;
  opacity: ${props => props.disabled ? 0.4 : 1};
  margin-left: auto;
  transition: color 0.2s, background-color 0.2s;

  &:hover {
    color: ${props => props.disabled ? palette.blue : palette.blueHover};
    background-color: ${props => props.disabled ? 'transparent' : palette.surfaceHover};
  }
`

const Panel = styled(TelemetryPanel)`
  width: ${TELEMETRY_EXPERIMENT_WIDTH}px;
  right: ${TELEMETRY_EXPERIMENT_RIGHT}px;
`

const VariableContentContainer = styled.div`
  display: flex;
  flex-direction: column;
`

const SectionSpacer = styled.div<{ $height?: number }>`
  height: ${props => props.$height ?? 8}px;
`

export const ExperimentStatePanel: React.FC = () => {
  const { experimentState } = useContext(ExperimentContext)

  const isExperimentOngoing = experimentState?.ongoing ?? false
  const isPaused = experimentState?.paused ?? false
  const isPausing = experimentState?.pause_requested ?? false
  const isElectron = !!(window as any).electronAPI

  const formatSeconds = (value?: number | null) => {
    if (value === undefined || value === null || value === 0) return '—'
    return `${value.toFixed(0)}s`
  }

  const handlePauseResume = () => {
    if (isPaused) {
      resumeExperimentRos(() => {
        console.log('Experiment resumed')
      })
    } else {
      pauseExperimentRos(() => {
        console.log('Experiment paused')
      })
    }
  }

  const handleDetachExperiment = async () => {
    const error = await (window as any).electronAPI?.toggleDetachedExperimentWindow()
    if (error) console.error('Failed to open detached window:', error)
  }

  const PauseResumeButton = isPaused ? StyledButton : StyledRedButton
  const pauseResumeLabel = isPaused ? 'Resume' : isPausing ? 'Pausing…' : 'Pause'

  const totalSteps = experimentState?.total_steps ?? 0
  const stepIndex = experimentState?.step_index ?? 0
  const stepLabel =
    (experimentState?.step_label && experimentState.step_label.length > 0)
      ? experimentState.step_label
      : (experimentState?.ongoing
          ? (experimentState.stage_name || experimentState.task_name || '—')
          : '—')
  const stepCountText =
    experimentState?.ongoing
      ? (totalSteps > 0
          ? `${stepIndex + 1} of ${totalSteps}`
          : `${(experimentState.stage_index ?? 0) + 1} of ${experimentState.total_stages ?? 0}`)
      : ''
  const STEP_TYPE_LABELS = ['Stage', 'Rest', 'Task'] as const
  const stepTypeText =
    experimentState?.ongoing
      ? (totalSteps > 0
          ? (STEP_TYPE_LABELS[experimentState.step_type as 0 | 1 | 2] ?? '—')
          : (experimentState.in_rest ? 'Rest' : experimentState.in_task ? 'Task' : 'Stage'))
      : '—'
  const showTrialProgress =
    !!experimentState?.ongoing &&
    !experimentState.in_rest &&
    !experimentState.in_task &&
    (totalSteps > 0 ? experimentState.step_type === 0 : true)

  return (
    <>
      <ExperimentStateTitle>
        <span>Experiment</span>
        <InfoIcon
          onClick={handleDetachExperiment}
          disabled={!isElectron}
          title={isElectron ? "Open detached experiment view" : "Only available in Electron"}
        >
          <FontAwesomeIcon icon={faWindowRestore} />
        </InfoIcon>
      </ExperimentStateTitle>
      <Panel>
        <StateRow>
          <StateTitle>Status</StateTitle>
          <StateValue>
            {experimentState?.ongoing
              ? (isPaused
                  ? 'Paused'
                  : isPausing
                    ? 'Pausing…'
                    : 'Running'
              ): 'Ready'}
          </StateValue>
        </StateRow>
        <SectionSpacer />
        <StateRow>
          <StateTitle>Stage</StateTitle>
          <StateValue>
            {experimentState?.ongoing ? stepLabel : '—'}
          </StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>&nbsp;</IndentedStateTitle>
          <StateValue>{stepCountText}</StateValue>
        </StateRow>
        <SectionSpacer />
        <StateRow>
          <StateTitle>Trial</StateTitle>
          <StateValue>
            {showTrialProgress ? `${experimentState.trial_in_stage + 1} of ${experimentState.total_trials_in_stage || 0}` : '—'}
          </StateValue>
        </StateRow>
        <StateRow>
          <StateTitle>Attempt</StateTitle>
          <StateValue>
            {showTrialProgress ? experimentState.attempt_in_trial + 1 : '—'}
          </StateValue>
        </StateRow>
        <SectionSpacer />
        <StateRow>
          <StateTitle>Failures</StateTitle>
          <StateValue>
            {experimentState?.ongoing ? experimentState.failed_pulses_in_session : '—'}
          </StateValue>
        </StateRow>
        <SectionSpacer />
        <StateRow>
          <StateTitle>Time</StateTitle>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Session</IndentedStateTitle>
          <StateValue>{formatSeconds(experimentState?.session_time)}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Experiment</IndentedStateTitle>
          <StateValue>{formatSeconds(experimentState?.experiment_time)}</StateValue>
        </StateRow>
        <VariableContentContainer>
          {experimentState?.in_rest ? (
            <>
              <StateRow>
                <IndentedStateTitle>Rest elapsed</IndentedStateTitle>
                <StateValue>{formatSeconds(experimentState.rest_elapsed)}</StateValue>
              </StateRow>
              <StateRow>
                <IndentedStateTitle>Rest remaining</IndentedStateTitle>
                <StateValue>{formatSeconds(experimentState.rest_remaining)}</StateValue>
              </StateRow>
            </>
          ) : experimentState?.in_task ? (
            <>
              <StateRow style={{ visibility: 'hidden' }}>
                <IndentedStateTitle>&nbsp;</IndentedStateTitle>
                <StateValue>&nbsp;</StateValue>
              </StateRow>
              <StateRow style={{ visibility: 'hidden' }}>
                <IndentedStateTitle>&nbsp;</IndentedStateTitle>
                <StateValue>&nbsp;</StateValue>
              </StateRow>
            </>
          ) : (
            <>
              <StateRow>
                <IndentedStateTitle>Stage elapsed</IndentedStateTitle>
                <StateValue>{formatSeconds(experimentState?.stage_elapsed_time)}</StateValue>
              </StateRow>
              <StateRow style={{ visibility: 'hidden' }}>
                <IndentedStateTitle>&nbsp;</IndentedStateTitle>
                <StateValue>&nbsp;</StateValue>
              </StateRow>
            </>
          )}
        </VariableContentContainer>
        <SectionSpacer $height={14} />
        <StateRow style={{ justifyContent: 'center' }}>
          <PauseResumeButton onClick={handlePauseResume} disabled={!isExperimentOngoing || isPausing}>
            {pauseResumeLabel}
          </PauseResumeButton>
        </StateRow>
      </Panel>
    </>
  )
}

