import React, { useContext, useEffect, useState } from 'react'
import styled from 'styled-components'

import { HealthcheckContext, ComponentHealth } from 'providers/HealthcheckProvider'
import { StyledPanel } from 'styles/General'

const HealthcheckMessagePanel = styled(StyledPanel)`
  width: 246px;
  height: 28px;
  position: fixed;
  top: 3px;
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
  const {
    eegBridgeStatus,
    eegSimulatorStatus,
    preprocessorStatus,
    deciderStatus,
    experimentCoordinatorStatus,
    resourceMonitorStatus,
  } = useContext(HealthcheckContext)

  let displayMessage

  // Prioritize unhealthy components in order of importance
  if (eegBridgeStatus.health === ComponentHealth.UNHEALTHY) {
    displayMessage = 'EEG Bridge unresponsive'
  } else if (eegSimulatorStatus.health === ComponentHealth.UNHEALTHY) {
    displayMessage = 'EEG Simulator unresponsive'
  } else if (preprocessorStatus.health === ComponentHealth.UNHEALTHY) {
    displayMessage = 'EEG Preprocessor unresponsive'
  } else if (deciderStatus.health === ComponentHealth.UNHEALTHY) {
    displayMessage = 'EEG Decider unresponsive'
  } else if (experimentCoordinatorStatus.health === ComponentHealth.UNHEALTHY) {
    displayMessage = 'Experiment Coordinator unresponsive'
  } else if (resourceMonitorStatus.health === ComponentHealth.UNHEALTHY) {
    displayMessage = 'Resource Monitor unresponsive'
  } else if (
    eegBridgeStatus.health === ComponentHealth.UNKNOWN ||
    eegSimulatorStatus.health === ComponentHealth.UNKNOWN ||
    preprocessorStatus.health === ComponentHealth.UNKNOWN ||
    deciderStatus.health === ComponentHealth.UNKNOWN ||
    experimentCoordinatorStatus.health === ComponentHealth.UNKNOWN ||
    resourceMonitorStatus.health === ComponentHealth.UNKNOWN
  ) {
    displayMessage = 'Waiting for component status...'
  } else {
    displayMessage = 'All systems operational'
  }

  return (
    <HealthcheckMessagePanel>
      <Header>Status</Header>
      {displayMessage && <Message>{displayMessage}</Message>}
    </HealthcheckMessagePanel>
  )
}
