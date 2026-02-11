import React, { useState, useEffect, useContext } from 'react'
import styled from 'styled-components'

import { StyledPanel, SmallerTitle, CONFIG_PANEL_WIDTH, StateRow, StateTitle, StateValue, StyledButton, StyledRedButton } from 'styles/General'
import { useSession, SessionStateValue } from 'providers/SessionProvider'
import { useSessionConfig } from 'providers/SessionConfigProvider'
import { RecordingContext } from 'providers/RecordingProvider'
import { EegSimulatorContext } from 'providers/EegSimulatorProvider'
import { LogContext } from 'providers/LogProvider'

const Container = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH}px;
  position: relative;
  margin-top: 0;
  margin-left: 0;
  left: 0;
`

const getStateDisplayText = (stateValue: SessionStateValue): string => {
  switch (stateValue) {
    case SessionStateValue.STOPPED:
      return 'Stopped'
    case SessionStateValue.INITIALIZING:
      return 'Initializing'
    case SessionStateValue.RUNNING:
      return 'Running'
    case SessionStateValue.FINALIZING:
      return 'Finalizing'
    default:
      return 'Unknown'
  }
}

export const SessionPanel: React.FC = () => {
  const { sessionState, startSession, abortSession } = useSession()
  const { dataSource } = useSessionConfig()
  const { recordingsList } = useContext(RecordingContext)
  const { datasetList } = useContext(EegSimulatorContext)
  const { clearAllLogs } = useContext(LogContext)
  const [displayedState, setDisplayedState] = useState(sessionState.state)

  /* Add 500ms hysteresis to prevent rapid flashing of state changes,
     as states (like INITIALIZING, FINALIZING) may sometimes change very quickly. */
  useEffect(() => {
    const timeoutId = setTimeout(() => {
      setDisplayedState(sessionState.state)
    }, 500)

    return () => clearTimeout(timeoutId)
  }, [sessionState.state])

  const handleStartSession = () => {
    // Clear pipeline logs before starting the session
    clearAllLogs()

    startSession((success: boolean, message?: string) => {
      if (success) {
        console.log('Session start requested successfully')
      } else {
        console.log('Failed to start session:', message)
      }
    })
  }

  const handleAbortSession = () => {
    abortSession((success: boolean) => {
      if (success) {
        console.log('Session abort requested successfully')
      } else {
        console.log('Failed to abort session')
      }
    })
  }

  const handleButtonClick = () => {
    if (sessionState.state === SessionStateValue.RUNNING) {
      handleAbortSession()
    } else {
      handleStartSession()
    }
  }

  const getButtonText = () => {
    if (sessionState.state === SessionStateValue.INITIALIZING) {
      return 'Starting...'
    }
    if (sessionState.state === SessionStateValue.FINALIZING) {
      return 'Stopping...'
    }
    if (sessionState.state === SessionStateValue.STOPPED) {
      return 'Start'
    }
    if (sessionState.state === SessionStateValue.RUNNING) {
      return 'Stop'
    }
    return 'Unknown'
  }

  // Disable button if no data is available for the selected data source
  const isNoDataAvailable = sessionState.state !== SessionStateValue.RUNNING && (
    (dataSource === 'recording' && recordingsList.length === 0) ||
    (dataSource === 'simulator' && datasetList.length === 0)
  )

  const isButtonDisabled = isNoDataAvailable ||
    sessionState.state === SessionStateValue.INITIALIZING ||
    sessionState.state === SessionStateValue.FINALIZING

  const ButtonComponent = sessionState.state === SessionStateValue.RUNNING ? StyledRedButton : StyledButton
  return (
    <Container>
      <SmallerTitle>Session</SmallerTitle>
      <StateRow>
        <StateTitle>Control:</StateTitle>
        <ButtonComponent
          onClick={handleButtonClick}
          disabled={isButtonDisabled}
          style={{ marginRight: '9px' }}
        >
          {getButtonText()}
        </ButtonComponent>
      </StateRow>

      <StateRow>
        <StateTitle>State:</StateTitle>
        <StateValue>{getStateDisplayText(displayedState)}</StateValue>
      </StateRow>
    </Container>
  )
}