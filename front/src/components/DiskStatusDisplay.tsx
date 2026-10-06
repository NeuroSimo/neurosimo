import React from 'react'
import styled from 'styled-components'

import { useDiskStatus, formatGiB, DiskSeverity } from 'providers/DiskStatusProvider'
import {
  StateRow,
  StateTitle,
  StateValue,
  palette,
} from 'styles/General'
import { STATUS_STRIP_HEIGHT, STATUS_STRIP_TOP, STATUS_MESSAGE_WIDTH } from 'components/HealthcheckMessageDisplay'

const DiskStatusPanel = styled.div`
  width: ${STATUS_MESSAGE_WIDTH}px;
  height: 28px;
  box-sizing: border-box;
  padding: 0 12px;
  display: flex;
  align-items: center;
  position: fixed;
  top: ${STATUS_STRIP_TOP + STATUS_STRIP_HEIGHT}px;
  right: 0;
  background-color: ${palette.surface};
  border-left: 1px solid ${palette.border};
  border-bottom: 1px solid ${palette.border};
  z-index: 1000;

  & > * {
    flex: 1;
    margin-bottom: 0;
  }
`

const StatusIndicator = styled.div<{ status: DiskSeverity }>`
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

export const DiskStatusDisplay: React.FC = () => {
  const { diskStatus, diskSeverity } = useDiskStatus()

  return (
    <>
      <DiskStatusPanel>
        <StateRow>
          <StateTitle>Free space:</StateTitle>
          <StateValue>
            <StatusIndicator status={diskSeverity} />
            {diskStatus ? `${formatGiB(diskStatus.free_bytes)} GiB` : '\u2013'}
          </StateValue>
        </StateRow>
      </DiskStatusPanel>
    </>
  )
}