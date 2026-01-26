import React, { useContext } from 'react'
import styled from 'styled-components'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'
import { faWindowRestore } from '@fortawesome/free-solid-svg-icons'

import {
  StyledPanel,
  ConfigRow,
  ConfigLabel,
  SmallerTitle,
  DASHBOARD_PANEL_OFFSET_FROM_TOP,
  DASHBOARD_PANEL_HEIGHT,
  StyledButton,
  StyledRedButton,
} from 'styles/General'
import { PipelineContext } from 'providers/PipelineProvider'
import { pauseExperimentRos, resumeExperimentRos } from 'ros/experiment'

const ExperimentStateTitle = styled.div`
  width: 255px;
  position: fixed;
  top: ${DASHBOARD_PANEL_OFFSET_FROM_TOP}px;
  right: 505px;
  z-index: 1001;
  display: flex;
  align-items: center;
  font-size: 12px;
  font-weight: bold;
`

const IconButton = styled.button<{ disabled: boolean }>`
  background: none;
  border: 0.5px solid #666666;
  border-radius: 3px;
  width: 18px;
  height: 18px;
  cursor: ${props => props.disabled ? 'not-allowed' : 'pointer'};
  display: flex;
  align-items: center;
  justify-content: center;
  color: #666666;
  opacity: ${props => props.disabled ? 0.5 : 1};
  font-size: 10px;
  margin-left: 188px;
`

const Panel = styled(StyledPanel)`
  width: 255px;
  height: ${DASHBOARD_PANEL_HEIGHT}px;
  position: fixed;
  top: ${DASHBOARD_PANEL_OFFSET_FROM_TOP + 20}px;
  right: 485px;
  z-index: 1000;
`

export const ExperimentStatePanel: React.FC = () => {
  const { experimentState } = useContext(PipelineContext)

  const isExperimentOngoing = experimentState?.ongoing ?? false
  const isPaused = experimentState?.paused ?? false
  const isElectron = !!(window as any).electronAPI

  const formatSeconds = (value?: number | null) => {
    if (value === undefined || value === null) return '—'
    return `${value.toFixed(1)}s`
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
    const error = await (window as any).electronAPI?.openDetachedExperimentWindow()
    if (error) console.error('Failed to open detached window:', error)
  }

  const PauseResumeButton = isPaused ? StyledButton : StyledRedButton
  const pauseResumeLabel = isPaused ? 'Resume' : 'Pause'

  return (
    <>
      <ExperimentStateTitle>
        <span>Experiment</span>
        <IconButton
          onClick={handleDetachExperiment}
          disabled={!isElectron}
          title={isElectron ? "Open detached experiment view" : "Only available in Electron"}
        >
          <FontAwesomeIcon icon={faWindowRestore} />
        </IconButton>
      </ExperimentStateTitle>
      <Panel>
        <ConfigRow>
          <ConfigLabel>Status:</ConfigLabel>
          <ConfigLabel>{experimentState?.ongoing ? (isPaused ? 'Paused' : 'Running') : 'Idle'}</ConfigLabel>
        </ConfigRow>
        <ConfigRow>
          <ConfigLabel>Stage:</ConfigLabel>
          <ConfigLabel>
            {experimentState?.stage_name
              ? `${experimentState.stage_name} (${(experimentState.stage_index ?? 0) + 1}/${experimentState.total_stages ?? 0})`
              : '—'}
          </ConfigLabel>
        </ConfigRow>
        <ConfigRow>
          <ConfigLabel>Trial:</ConfigLabel>
          <ConfigLabel>
            {experimentState ? `${experimentState.trial}/${experimentState.total_trials_in_stage || 0}` : '—'}
          </ConfigLabel>
        </ConfigRow>
        <ConfigRow>
          <ConfigLabel>Experiment time:</ConfigLabel>
          <ConfigLabel>{formatSeconds(experimentState?.experiment_time)}</ConfigLabel>
        </ConfigRow>
        <ConfigRow>
          <ConfigLabel>Stage elapsed:</ConfigLabel>
          <ConfigLabel>{formatSeconds(experimentState?.stage_elapsed_time)}</ConfigLabel>
        </ConfigRow>
        <br />
        <ConfigRow style={{ justifyContent: 'center', paddingRight: 12 }}>
          <PauseResumeButton onClick={handlePauseResume} disabled={!isExperimentOngoing}>
            {pauseResumeLabel}
          </PauseResumeButton>
        </ConfigRow>
      </Panel>
    </>
  )
}

