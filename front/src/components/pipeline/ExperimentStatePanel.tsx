import React, { useContext } from 'react'
import styled from 'styled-components'

import {
  StyledPanel,
  ConfigRow,
  ConfigLabel,
  SmallerTitle,
  DASHBOARD_PANEL_OFFSET_FROM_TOP,
  DASHBOARD_PANEL_HEIGHT,
} from 'styles/General'
import { PipelineContext } from 'providers/PipelineProvider'

const ExperimentStateTitle = styled.div`
  width: 210px;
  position: fixed;
  top: ${DASHBOARD_PANEL_OFFSET_FROM_TOP}px;
  right: 480px;
  z-index: 1001;
  text-align: left;
  font-size: 12px;
  font-weight: bold;
`

const Panel = styled(StyledPanel)`
  width: 185px;
  height: ${DASHBOARD_PANEL_HEIGHT}px;
  position: fixed;
  top: ${DASHBOARD_PANEL_OFFSET_FROM_TOP + 20}px;
  right: 480px;
  z-index: 1000;
`

export const ExperimentStatePanel: React.FC = () => {
  const { experimentState } = useContext(PipelineContext)

  const formatSeconds = (value?: number | null) => {
    if (value === undefined || value === null) return '—'
    return `${value.toFixed(1)}s`
  }

  return (
    <>
      <ExperimentStateTitle>Experiment</ExperimentStateTitle>
      <Panel>
        <ConfigRow>
          <ConfigLabel>Status:</ConfigLabel>
          <ConfigLabel>{experimentState?.ongoing ? 'Running' : 'Idle'}</ConfigLabel>
        </ConfigRow>
        <ConfigRow>
          <ConfigLabel>Stage:</ConfigLabel>
          <ConfigLabel>
            {experimentState?.current_stage_name
              ? `${experimentState.current_stage_name} (${(experimentState.current_stage_index ?? 0) + 1}/${experimentState.total_stages ?? 0})`
              : '—'}
          </ConfigLabel>
        </ConfigRow>
        <ConfigRow>
          <ConfigLabel>Trial:</ConfigLabel>
          <ConfigLabel>
            {experimentState ? `${experimentState.current_trial}/${experimentState.total_trials_in_stage || 0}` : '—'}
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
      </Panel>
    </>
  )
}

