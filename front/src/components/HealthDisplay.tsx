import React, { useContext } from 'react'
import styled from 'styled-components'

import { HealthcheckContext, ComponentHealth } from 'providers/HealthProvider'
import { StyledPanel } from 'styles/General'

interface StatusSquareProps {
  status: string
}

const HealthPanel = styled(StyledPanel)`
  width: 200px;
  height: 25px;
  position: fixed;
  top: 6px;
  right: 280px;
  display: grid;
  grid-template-columns: 1fr 1fr;
  grid-gap: 6px;
  z-index: 1000;
`

const StatusSquare = styled.div<StatusSquareProps>`
  width: 10px;
  height: 10px;
  display: inline-block;
  vertical-align: middle;
  background-color: ${({ status }) => {
    switch (status) {
      case ComponentHealth.READY:
        return 'green'
      case ComponentHealth.DEGRADED:
        return 'yellow'
      case ComponentHealth.ERROR:
        return 'red'
      case ComponentHealth.UNKNOWN:
      default:
        return 'grey'
    }
  }};
  border: 2px solid black;
  margin-right: 6px;
`

const StatusLine = styled.div`
  font-size: 0.7rem;
  font-weight: bold;
  margin-bottom: 5px;
`

export const HealthDisplay: React.FC = () => {
  const { eegBridgeStatus, preprocessorStatus, deciderStatus } =
    useContext(HealthcheckContext)

  return (
    <HealthPanel>
      <StatusLine>
        <StatusSquare status={preprocessorStatus?.health || ComponentHealth.UNKNOWN} />
        Preprocessor
      </StatusLine>
      <StatusLine>
        <StatusSquare status={eegBridgeStatus?.health || ComponentHealth.UNKNOWN} />
        EEG
      </StatusLine>
      <StatusLine>
        <StatusSquare status={deciderStatus?.health || ComponentHealth.UNKNOWN} />
        Decider
      </StatusLine>
    </HealthPanel>
  )
}
