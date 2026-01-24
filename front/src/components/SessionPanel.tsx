import React, { useState, useEffect } from 'react'
import styled from 'styled-components'

import { StyledPanel, SmallerTitle, CONFIG_PANEL_WIDTH, StateRow, StateTitle, StateValue, StyledButton, StyledRedButton } from 'styles/General'
import { useSession } from 'providers/SessionProvider'
import { SessionStage } from 'ros/session'

const Container = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH}px;
  position: relative;
  margin-top: 0;
  margin-left: 0;
  left: 0;
`

const getStageDisplayText = (stage: SessionStage): string => {
  switch (stage) {
    case SessionStage.STOPPED:
      return 'Stopped'
    case SessionStage.INITIALIZING:
      return 'Initializing'
    case SessionStage.RUNNING:
      return 'Running'
    case SessionStage.FINALIZING:
      return 'Finalizing'
    default:
      return 'Unknown'
  }
}

export const SessionPanel: React.FC = () => {
  const { sessionState, startSession, abortSession } = useSession()
  const [isLoading, setIsLoading] = useState(false)
  const [displayedStage, setDisplayedStage] = useState(sessionState.stage)

  /* Add 500ms hysteresis to prevent rapid flashing of stage changes,
     as stages (like INITIALIZING, FINALIZING) may sometimes change very quickly. */
  useEffect(() => {
    const timeoutId = setTimeout(() => {
      setDisplayedStage(sessionState.stage)
    }, 500)

    return () => clearTimeout(timeoutId)
  }, [sessionState.stage])

  const handleStartSession = () => {
    setIsLoading(true)

    startSession((success: boolean, message?: string) => {
      setIsLoading(false)
      if (success) {
        console.log('Session start requested successfully')
      } else {
        console.log('Failed to start session:', message)
      }
    })
  }

  const handleAbortSession = () => {
    setIsLoading(true)

    abortSession((success: boolean, message?: string) => {
      setIsLoading(false)
      if (success) {
        console.log('Session abort requested successfully')
      } else {
        console.log('Failed to abort session:', message)
      }
    })
  }

  const handleButtonClick = () => {
    if (sessionState.isRunning) {
      handleAbortSession()
    } else {
      handleStartSession()
    }
  }

  const getButtonText = () => {
    if (isLoading) {
      return sessionState.isRunning ? 'Stopping...' : 'Starting...'
    }
    return sessionState.isRunning ? 'Stop' : 'Start'
  }

  const ButtonComponent = sessionState.isRunning ? StyledRedButton : StyledButton

  return (
    <Container>
      <SmallerTitle>Session</SmallerTitle>
      <StateRow>
        <StateTitle>Control:</StateTitle>
        <ButtonComponent
          onClick={handleButtonClick}
          disabled={isLoading}
          style={{ marginRight: '9px' }}
        >
          {getButtonText()}
        </ButtonComponent>
      </StateRow>

      <StateRow>
        <StateTitle>Stage:</StateTitle>
        <StateValue>{getStageDisplayText(displayedStage)}</StateValue>
      </StateRow>
    </Container>
  )
}