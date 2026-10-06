import React, { useContext } from 'react'
import styled from 'styled-components'

import { HealthcheckContext, ComponentHealth } from 'providers/HealthProvider'
import { palette } from 'styles/General'
import { STATUS_STRIP_HEIGHT, STATUS_STRIP_TOP, STATUS_MESSAGE_WIDTH } from 'components/HealthcheckMessageDisplay'

interface StatusSquareProps {
  status: string
}

const HealthPanel = styled.div`
  width: 221px;
  height: ${STATUS_STRIP_HEIGHT}px;
  box-sizing: border-box;
  padding: 10px 12px;
  position: fixed;
  top: ${STATUS_STRIP_TOP}px;
  right: ${STATUS_MESSAGE_WIDTH}px;
  display: grid;
  grid-template-columns: 1fr 1fr;
  grid-gap: 4px 8px;
  align-content: start;
  background-color: ${palette.surface};
  border-left: 1px solid ${palette.border};
  border-bottom: 1px solid ${palette.border};
  z-index: 1000;
`

const StatusSquare = styled.div<StatusSquareProps>`
  width: 8px;
  height: 8px;
  border-radius: 2px;
  display: inline-block;
  vertical-align: middle;
  background-color: ${({ status }) => {
    switch (status) {
      case ComponentHealth.READY:
        return palette.green
      case ComponentHealth.DEGRADED:
        return palette.yellow
      case ComponentHealth.ERROR:
        return palette.red
      case ComponentHealth.UNKNOWN:
      default:
        return palette.textDim
    }
  }};
  margin-right: 7px;
  position: relative;
  top: -1px;
`

const StatusLine = styled.div`
  font-size: 12px;
  font-weight: 500;
  color: ${palette.textSecondary};
  white-space: nowrap;
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
