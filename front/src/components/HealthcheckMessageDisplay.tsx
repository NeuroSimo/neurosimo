import React, { useContext, useEffect, useState } from 'react'
import styled from 'styled-components'

import { HealthcheckContext, ComponentHealth } from 'providers/HealthProvider'
import { SessionContext } from 'providers/SessionProvider'
import { palette, TELEMETRY_EXPERIMENT_RIGHT } from 'styles/General'

/* Geometry of the top-right status strip (health, status message, disk). It hangs from the menu bar
   and shares its column edges with the telemetry panels below: health sits above Experiment,
   status message and free space above Statistics and Stimulation. */
export const STATUS_STRIP_TOP = 30
export const STATUS_STRIP_HEIGHT = 56
export const DISK_ROW_HEIGHT = 28
export const STATUS_MESSAGE_WIDTH = TELEMETRY_EXPERIMENT_RIGHT

const HealthcheckMessagePanel = styled.div`
  width: ${STATUS_MESSAGE_WIDTH}px;
  height: ${STATUS_STRIP_HEIGHT}px;
  box-sizing: border-box;
  padding: 8px 12px;
  position: fixed;
  top: ${STATUS_STRIP_TOP}px;
  right: 0;
  overflow: hidden;
  background-color: ${palette.surface};
  border-left: 1px solid ${palette.border};
  border-bottom: 1px solid ${palette.border};
  z-index: 1000;
`

const Header = styled.div`
  color: ${palette.textSecondary};
  font-weight: 600;
  font-size: 11px;
  letter-spacing: 0.06em;
  text-transform: uppercase;
  margin-bottom: 3px;
`

const Message = styled.div`
  color: ${palette.text};
  font-size: 12px;
  line-height: 1.35;
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

  const { sessionState } = useContext(SessionContext)

  let displayMessage

  // Highest priority: session abort reason
  if (sessionState.abortReason) {
    displayMessage = `Session aborted: ${sessionState.abortReason}`
  }
  // Prioritize error states, then degraded states, then unknown states
  else if (eegBridgeStatus.health === ComponentHealth.ERROR) {
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
    displayMessage = 'System operational'
  }

  return (
    <HealthcheckMessagePanel>
      <Header>Status</Header>
      {displayMessage && <Message>{displayMessage}</Message>}
    </HealthcheckMessagePanel>
  )
}
