import React, { useContext } from 'react'
import styled from 'styled-components'

import { StyledPanel, SmallerTitle, ConfigRow, ConfigLabel, Select, CONFIG_PANEL_WIDTH } from 'styles/General'
import { setParameterRos } from 'ros/parameters'
import { PipelineContext } from 'providers/PipelineProvider'

const Container = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH}px;
  position: relative;
  margin-top: 0;
  margin-left: 0;
  left: 0;
`

export const ExperimentPanel: React.FC = () => {
  const { protocolName, protocolList, experimentState } = useContext(PipelineContext)

  const isExperimentOngoing = experimentState?.ongoing ?? false

  const handleProtocolChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const protocol = event.target.value
    setParameterRos('experiment.protocol', protocol, () => {
      console.log('Protocol set to ' + protocol)
    })
  }

  return (
    <Container>
      <SmallerTitle>Experiment</SmallerTitle>
      <ConfigRow>
        <ConfigLabel>Protocol:</ConfigLabel>
        <Select onChange={handleProtocolChange} value={protocolName} disabled={isExperimentOngoing}>
          {protocolList.map((protocol, index) => (
            <option key={index} value={protocol}>
              {protocol}
            </option>
          ))}
        </Select>
      </ConfigRow>
    </Container>
  )
}

