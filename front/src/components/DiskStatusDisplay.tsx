import React, { useContext } from 'react'
import styled from 'styled-components'

import { DiskStatusContext } from 'providers/DiskStatusProvider'
import {
  StyledPanel,
  StateRow,
  StateTitle,
  StateValue,
  DASHBOARD_PANEL_OFFSET_FROM_TOP,
  DASHBOARD_PANEL_HEIGHT,
} from 'styles/General'

const DISK_STATUS_PANEL_OFFSET_FROM_TOP = 110

const DiskStatusPanel = styled(StyledPanel)`
  width: 185px;
  height: 10px;
  position: fixed;
  top: ${DISK_STATUS_PANEL_OFFSET_FROM_TOP}px;
  right: 3px;
  z-index: 1000;
`

const StatusIndicator = styled.div<{ status: 'ok' | 'warning' | 'error' }>`
  width: 12px;
  height: 12px;
  border-radius: 50%;
  display: inline-block;
  vertical-align: middle;
  background-color: ${({ status }) => {
    switch (status) {
      case 'ok':
        return 'green'
      case 'warning':
        return 'yellow'
      case 'error':
        return 'red'
      default:
        return 'grey'
    }
  }};
  border: 2px solid black;
  margin-right: 8px;
`

export const DiskStatusDisplay: React.FC = () => {
  const { diskStatus } = useContext(DiskStatusContext)

  const getStatus = (): 'ok' | 'warning' | 'error' => {
    if (!diskStatus) return 'error'
    if (diskStatus.free_bytes < diskStatus.error_threshold_bytes) return 'error'
    if (diskStatus.free_bytes < diskStatus.warning_threshold_bytes) return 'warning'
    return 'ok'
  }

  const formatFreeSpace = (bytes: number): string => {
    const gib = bytes / (1024 ** 3)
    return gib.toFixed(1)
  }

  return (
    <>
      <DiskStatusPanel>
        <StateRow>
          <StateTitle>Free space:</StateTitle>
          <StateValue>
            <StatusIndicator status={getStatus()} />
            {diskStatus ? `${formatFreeSpace(diskStatus.free_bytes)} GiB` : '\u2013'}
          </StateValue>
        </StateRow>
      </DiskStatusPanel>
    </>
  )
}