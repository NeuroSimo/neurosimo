import React, { useContext } from 'react'
import styled from 'styled-components'

import { StyledPanel, SmallerTitle, ConfigRow, ConfigLabel, Select, CONFIG_PANEL_WIDTH } from 'styles/General'
import { useParameters } from 'providers/ParameterProvider'
import { PipelineContext } from 'providers/PipelineProvider'
import { useSession, SessionStage } from 'providers/SessionProvider'

const Container = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH}px;
  position: relative;
  margin-top: 0;
  margin-left: 0;
  left: 0;
`

export const ExperimentPanel: React.FC = () => {
  const { protocolName, protocolList } = useContext(PipelineContext)
  const { setExperimentProtocol } = useParameters()
  const { sessionState } = useSession()

  const isSessionRunning = sessionState.stage !== SessionStage.STOPPED

  const handleProtocolChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const protocol = event.target.value
    setExperimentProtocol(protocol, () => {
      console.log('Protocol set to ' + protocol)
    })
  }

  return (
    <Container>
      <SmallerTitle>Experiment</SmallerTitle>
      <ConfigRow>
        <ConfigLabel>Protocol:</ConfigLabel>
        <Select onChange={handleProtocolChange} value={protocolName} disabled={isSessionRunning}>
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

