import React, { useContext } from 'react'
import styled from 'styled-components'

import { HealthcheckContext, HealthcheckStatus } from 'providers/HealthcheckProvider'
import { StyledPanel } from 'styles/General'

interface StatusSquareProps {
  status: number
}

const HealthcheckPanel = styled(StyledPanel)`
  width: 300px;
  height: 40px;
  position: fixed;
  top: 10px;
  right: 5px;
  display: grid;
  grid-template-columns: 1fr 1fr;
  grid-gap: 10px;
  z-index: 1000;
`

const StatusSquare = styled.div<StatusSquareProps>`
  width: 16px;
  height: 16px;
  display: inline-block;
  vertical-align: middle;
  background-color: ${({ status }) => {
    switch (status) {
      case HealthcheckStatus.READY:
        return 'green'
      case HealthcheckStatus.NOT_READY:
        return 'yellow'
      case HealthcheckStatus.DISABLED:
        return 'orange'
      case HealthcheckStatus.ERROR:
        return 'red'
      default:
        return 'grey'
    }
  }};
  border: 3px solid black;
  margin-right: 10px;
`

const StatusLine = styled.div`
  font-size: 0.9rem;
  font-weight: bold;
  margin-bottom: 8px;
`

export const HealthcheckStatusDisplay: React.FC = () => {
  const { eegHealthcheck, mtmsDeviceHealthcheck, preprocessorHealthcheck, deciderHealthcheck } =
    useContext(HealthcheckContext)

  return (
    <HealthcheckPanel>
      {mtmsDeviceHealthcheck?.status.value !== HealthcheckStatus.DISABLED && (
        <StatusLine>
          <StatusSquare
            status={
              mtmsDeviceHealthcheck?.status.value !== undefined && mtmsDeviceHealthcheck?.status.value !== null
                ? mtmsDeviceHealthcheck?.status.value
                : -1
            }
          />
          mTMS device
        </StatusLine>
      )}
      <StatusLine>
        <StatusSquare
          status={
            preprocessorHealthcheck?.status.value !== undefined && preprocessorHealthcheck?.status.value !== null
              ? preprocessorHealthcheck?.status.value
              : -1
          }
        />
        Preprocessor
      </StatusLine>
      <StatusLine>
        <StatusSquare
          status={
            eegHealthcheck?.status.value !== undefined && eegHealthcheck?.status.value !== null
              ? eegHealthcheck?.status.value
              : -1
          }
        />
        EEG
      </StatusLine>
      <StatusLine>
        <StatusSquare
          status={
            deciderHealthcheck?.status.value !== undefined && deciderHealthcheck?.status.value !== null
              ? deciderHealthcheck?.status.value
              : -1
          }
        />
        Decider
      </StatusLine>
    </HealthcheckPanel>
  )
}
