import React, { useContext } from 'react'
import styled from 'styled-components'

import { StyledPanel, SmallerTitle, ConfigRow, ConfigLabel, Select } from 'styles/General'
import { setExperimentProtocolRos } from 'ros/pipeline'
import { PipelineContext } from 'providers/PipelineProvider'

const Container = styled(StyledPanel)`
  width: 185px;
  position: relative;
  margin-top: -117px;
  margin-left: 80px;
  left: 10px;
`

export const ExperimentPanel: React.FC = () => {
  const { protocolName, protocolList, experimentState } = useContext(PipelineContext)

  const handleProtocolChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const protocol = event.target.value
    setExperimentProtocolRos(protocol, () => {
      console.log('Protocol set to ' + protocol)
    })
  }

  const formatSeconds = (value?: number) => {
    if (value === undefined || value === null) return '—'
    return `${value.toFixed(1)}s`
  }

  return (
    <Container>
      <SmallerTitle>Experiment</SmallerTitle>
      <ConfigRow>
        <ConfigLabel>Protocol:</ConfigLabel>
        <Select onChange={handleProtocolChange} value={protocolName}>
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

