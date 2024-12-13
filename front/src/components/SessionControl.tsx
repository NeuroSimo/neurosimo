import React, { useContext } from 'react'

import { startSession, stopSession } from 'ros/services/session'
import { StyledButton } from 'styles/General'

import { SystemContext, SessionState, DeviceState } from 'providers/SystemProvider'

export const SessionControl = () => {
  const { session } = useContext(SystemContext)

  const sessionState = session?.state

  const sessionText = () => {
    if (sessionState?.value === SessionState.STOPPED) return 'Start'
    if (sessionState?.value === SessionState.STARTING) return 'Starting'
    if (sessionState?.value === SessionState.STARTED) return 'Stop'
    if (sessionState?.value === SessionState.STOPPING) return 'Stopping'
    return '???'
  }

  const toggleSession = () => {
    if (sessionState?.value === SessionState.STOPPED) {
      startSession()
    } else if (sessionState?.value === SessionState.STARTED) {
      stopSession()
    }
  }

  return (
    <>
      <StyledButton
        onClick={toggleSession}
        disabled={sessionState?.value === SessionState.STARTING || sessionState?.value === SessionState.STOPPING}
      >
        {sessionText()} session
      </StyledButton>
    </>
  )
}
