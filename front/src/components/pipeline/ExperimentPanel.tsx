import React from 'react'
import styled from 'styled-components'

import { StyledPanel, SmallerTitle, ConfigRow, ConfigLabel, Select } from 'styles/General'

type Props = {
  protocolName: string
  protocolList: string[]
  experimentState: {
    ongoing?: boolean
    current_stage_name?: string | null
    current_stage_index?: number | null
    total_stages?: number | null
    current_trial?: number | null
    total_trials_in_stage?: number | null
    experiment_time?: number | null
    stage_elapsed_time?: number | null
  } | null
  onProtocolChange: (event: React.ChangeEvent<HTMLSelectElement>) => void
  formatSeconds: (value?: number) => string
}

const Container = styled(StyledPanel)`
  width: 185px;
  position: relative;
  margin-top: -117px;
  margin-left: 80px;
  left: 10px;
`

export const ExperimentPanel: React.FC<Props> = ({ protocolName, protocolList, experimentState, onProtocolChange, formatSeconds }) => {
  return (
    <Container>
      <SmallerTitle>Experiment</SmallerTitle>
      <ConfigRow>
        <ConfigLabel>Protocol:</ConfigLabel>
        <Select onChange={onProtocolChange} value={protocolName}>
          {protocolList.map((protocol, index) => (
            <option key={index} value={protocol}>
              {protocol}
            </option>
          ))}
        </Select>
      </ConfigRow>
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
        <ConfigLabel>{formatSeconds(experimentState?.experiment_time ?? undefined)}</ConfigLabel>
      </ConfigRow>
      <ConfigRow>
        <ConfigLabel>Stage elapsed:</ConfigLabel>
        <ConfigLabel>{formatSeconds(experimentState?.stage_elapsed_time ?? undefined)}</ConfigLabel>
      </ConfigRow>
    </Container>
  )
}

