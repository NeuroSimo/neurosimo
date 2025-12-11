import React, { useContext, useEffect, useState } from 'react'
import styled from 'styled-components'

import { HealthcheckContext, HealthcheckStatus } from 'providers/HealthcheckProvider'
import { StyledPanel } from 'styles/General'

const HealthcheckMessagePanel = styled(StyledPanel)`
  width: 246px;
  height: 40px;
  position: fixed;
  top: 75px;
  right: 3px;
  z-index: 1000;
`

const Header = styled.div`
  color: #333;
  font-weight: bold;
  font-size: 0.8rem;
  margin-bottom: 0.31rem;
`

const Message = styled.div`
  color: #333;
  font-size: 0.7rem;
  padding: 3px;
  border-bottom: 0px;
  transition: opacity 0.3s;
`

export const HealthcheckMessageDisplay: React.FC = () => {
  const { eegHealthcheck, preprocessorHealthcheck, deciderHealthcheck } =
    useContext(HealthcheckContext)

  let displayMessage

  // Prioritize eegHealthcheck > preprocessorHealthcheck > deciderHealthcheck
  if (eegHealthcheck?.status.value !== HealthcheckStatus.READY) {
    displayMessage = eegHealthcheck?.actionable_message
  } else if (
    preprocessorHealthcheck?.status.value === HealthcheckStatus.DISABLED ||
    preprocessorHealthcheck?.status.value === HealthcheckStatus.ERROR
  ) {
    displayMessage = preprocessorHealthcheck?.actionable_message
  } else if (
    deciderHealthcheck?.status.value === HealthcheckStatus.DISABLED ||
    deciderHealthcheck?.status.value === HealthcheckStatus.ERROR
  ) {
    displayMessage = deciderHealthcheck?.actionable_message
  } else {
    displayMessage = 'Ready'
  }

  return (
    <HealthcheckMessagePanel>
      <Header>Status</Header>
      {displayMessage && <Message>{displayMessage}</Message>}
    </HealthcheckMessagePanel>
  )
}
