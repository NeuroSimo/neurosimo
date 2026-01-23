import React, { useState } from 'react'
import styled from 'styled-components'

import { StyledPanel, SmallerTitle, CONFIG_PANEL_WIDTH, StateRow, StateTitle, StateValue, StyledButton, StyledRedButton } from 'styles/General'
import { useSession } from 'providers/SessionProvider'

const Container = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH}px;
  position: relative;
  margin-top: 0;
  margin-left: 0;
  left: 0;
`

const SessionStatus = styled.div`
  font-size: 10px;
  font-weight: bold;
  color: #666;
  text-align: center;
  margin-top: 4px;
  padding: 2px;
`

export const SessionPanel: React.FC = () => {
  const { sessionState, startSession, stopSession } = useSession()
  const [isLoading, setIsLoading] = useState(false)

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

  const handleStopSession = () => {
    setIsLoading(true)

    stopSession((success: boolean, message?: string) => {
      setIsLoading(false)
      if (success) {
        console.log('Session stop requested successfully')
      } else {
        console.log('Failed to stop session:', message)
      }
    })
  }

  const handleButtonClick = () => {
    if (sessionState.isRunning) {
      handleStopSession()
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

      {sessionState.phase && (
        <SessionStatus>
          Phase: {sessionState.phase}
        </SessionStatus>
      )}
    </Container>
  )
}