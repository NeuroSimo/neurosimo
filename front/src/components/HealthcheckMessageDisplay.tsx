import React, { useContext, useEffect, useState } from 'react'
import styled from 'styled-components'

import { HealthcheckContext, ComponentHealth } from 'providers/HealthProvider'
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
    presenterStatus,
    triggerTimerStatus,
  } = useContext(HealthcheckContext)

  let displayMessage

  // Prioritize error states, then degraded states, then unknown states
  if (eegBridgeStatus.health === ComponentHealth.ERROR) {
    displayMessage = eegBridgeStatus.message || 'EEG Bridge error'
  } else if (eegSimulatorStatus.health === ComponentHealth.ERROR) {
    displayMessage = eegSimulatorStatus.message || 'EEG Simulator error'
  } else if (preprocessorStatus.health === ComponentHealth.ERROR) {
    displayMessage = preprocessorStatus.message || 'EEG Preprocessor error'
  } else if (deciderStatus.health === ComponentHealth.ERROR) {
    displayMessage = deciderStatus.message || 'EEG Decider error'
  } else if (experimentCoordinatorStatus.health === ComponentHealth.ERROR) {
    displayMessage = experimentCoordinatorStatus.message || 'Experiment Coordinator error'
  } else if (resourceMonitorStatus.health === ComponentHealth.ERROR) {
    displayMessage = resourceMonitorStatus.message || 'Resource Monitor error'
  } else if (presenterStatus.health === ComponentHealth.ERROR) {
    displayMessage = presenterStatus.message || 'Presenter error'
  } else if (triggerTimerStatus.health === ComponentHealth.ERROR) {
    displayMessage = triggerTimerStatus.message || 'Trigger Timer error'
  } else if (eegBridgeStatus.health === ComponentHealth.DEGRADED) {
    displayMessage = eegBridgeStatus.message || 'EEG Bridge degraded'
  } else if (eegSimulatorStatus.health === ComponentHealth.DEGRADED) {
    displayMessage = eegSimulatorStatus.message || 'EEG Simulator degraded'
  } else if (preprocessorStatus.health === ComponentHealth.DEGRADED) {
    displayMessage = preprocessorStatus.message || 'EEG Preprocessor degraded'
  } else if (deciderStatus.health === ComponentHealth.DEGRADED) {
    displayMessage = deciderStatus.message || 'EEG Decider degraded'
  } else if (experimentCoordinatorStatus.health === ComponentHealth.DEGRADED) {
    displayMessage = experimentCoordinatorStatus.message || 'Experiment Coordinator degraded'
  } else if (resourceMonitorStatus.health === ComponentHealth.DEGRADED) {
    displayMessage = resourceMonitorStatus.message || 'Resource Monitor degraded'
  } else if (presenterStatus.health === ComponentHealth.DEGRADED) {
    displayMessage = presenterStatus.message || 'Presenter degraded'
  } else if (triggerTimerStatus.health === ComponentHealth.DEGRADED) {
    displayMessage = triggerTimerStatus.message || 'Trigger Timer degraded'
  } else if (eegBridgeStatus.health === ComponentHealth.UNKNOWN) {
    displayMessage = 'EEG Bridge unresponsive'
  } else if (eegSimulatorStatus.health === ComponentHealth.UNKNOWN) {
    displayMessage = 'EEG Simulator unresponsive'
  } else if (preprocessorStatus.health === ComponentHealth.UNKNOWN) {
    displayMessage = 'EEG Preprocessor unresponsive'
  } else if (deciderStatus.health === ComponentHealth.UNKNOWN) {
    displayMessage = 'EEG Decider unresponsive'
  } else if (experimentCoordinatorStatus.health === ComponentHealth.UNKNOWN) {
    displayMessage = 'Experiment Coordinator unresponsive'
  } else if (resourceMonitorStatus.health === ComponentHealth.UNKNOWN) {
    displayMessage = 'Resource Monitor unresponsive'
  } else if (presenterStatus.health === ComponentHealth.UNKNOWN) {
    displayMessage = 'Presenter unresponsive'
  } else if (triggerTimerStatus.health === ComponentHealth.UNKNOWN) {
    displayMessage = 'Trigger Timer unresponsive'
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
