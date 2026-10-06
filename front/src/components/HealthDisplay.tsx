import React, { useContext } from 'react'
import styled from 'styled-components'

import { HealthcheckContext, ComponentHealth } from 'providers/HealthProvider'
import { useDiskStatus, formatGiB, DiskSeverity } from 'providers/DiskStatusProvider'
import { palette, StateRow, StateTitle, StateValue, TELEMETRY_EXPERIMENT_WIDTH } from 'styles/General'
import {
  STATUS_STRIP_HEIGHT,
  STATUS_STRIP_TOP,
  STATUS_MESSAGE_WIDTH,
} from 'components/HealthcheckMessageDisplay'

interface StatusSquareProps {
  status: string
}

const HealthPanel = styled.div`
  width: ${TELEMETRY_EXPERIMENT_WIDTH}px;
  height: ${STATUS_STRIP_HEIGHT}px;
  box-sizing: border-box;
  padding: 10px 12px;
  position: fixed;
  top: ${STATUS_STRIP_TOP}px;
  right: ${STATUS_MESSAGE_WIDTH}px;
  display: flex;
  flex-direction: column;
  background-color: ${palette.surface};
  border-left: 1px solid ${palette.border};
  border-bottom: 1px solid ${palette.border};
  z-index: 1000;
`

const ComponentGrid = styled.div`
  display: grid;
  grid-template-columns: 1fr 1fr;
  grid-gap: 4px 8px;
  align-content: start;
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

/* Free space is a resource metric rather than a component check, so it keeps its own row
   treatment and sits apart from the component grid, at the bottom of the panel. */
const DiskRow = styled(StateRow)`
  margin-top: auto;
  margin-bottom: 0;
`

const DiskIndicator = styled.div<{ status: DiskSeverity }>`
  width: 8px;
  height: 8px;
  border-radius: 50%;
  display: inline-block;
  vertical-align: middle;
  background-color: ${({ status }) => {
    switch (status) {
      case 'ok':
        return palette.green
      case 'warning':
        return palette.yellow
      case 'error':
        return palette.red
      default:
        return palette.textDim
    }
  }};
  margin-right: 7px;
  position: relative;
  top: -1px;
`

export const HealthDisplay: React.FC = () => {
  const { eegBridgeStatus, preprocessorStatus, deciderStatus } =
    useContext(HealthcheckContext)
  const { diskStatus, diskSeverity } = useDiskStatus()

  return (
    <HealthPanel>
      <ComponentGrid>
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
      </ComponentGrid>
      <DiskRow>
        <StateTitle>Free space:</StateTitle>
        <StateValue>
          <DiskIndicator status={diskSeverity} />
          {diskStatus ? `${formatGiB(diskStatus.free_bytes)} GiB` : '\u2013'}
        </StateValue>
      </DiskRow>
    </HealthPanel>
  )
}
