import React, { useState, useEffect, useContext } from 'react'
import styled from 'styled-components'

import { ConfigPanel, ConfigTitle, StateRow, StateTitle, StateValue, StyledButton, StyledRedButton, palette } from 'styles/General'
import { useSession, SessionStateValue } from 'providers/SessionProvider'
import { useSessionConfig } from 'providers/SessionConfigProvider'
import { ModuleListContext } from 'providers/ModuleListProvider'
import { RecordingContext } from 'providers/RecordingProvider'
import { EegSimulatorContext } from 'providers/EegSimulatorProvider'
import { LogContext } from 'providers/LogProvider'
import { useDiskStatus, getDiskSeverity, formatGiB } from 'providers/DiskStatusProvider'

/* A boxed panel on a raised surface, so that the primary run control stands out from the ordinary
   sidebar sections. It spans the sidebar's content column and is pinned to the bottom of the
   sidebar, so it stays put when the data source tab changes and grows upwards with its banners. */
const Container = styled(ConfigPanel)`
  width: auto;
  position: relative;
  margin: auto 16px 16px 20px;
  padding: 10px 15px 10px 14px;
  background-color: ${palette.surfaceRaised};
  border: 1px solid ${palette.borderStrong};
  border-radius: 3px;
  flex-shrink: 0;
`

/* Deliberately loud: a low-disk condition must not be mistakable for ordinary status text. */
const WarningBanner = styled.div`
  display: flex;
  align-items: flex-start;
  gap: 8px;
  margin: 0 0 10px 0;
  padding: 8px 10px;
  border: 1px solid ${palette.yellow};
  border-left: 4px solid ${palette.yellow};
  border-radius: 3px;
  background-color: rgba(216, 180, 69, 0.12);
  color: #ead48f;
  font-size: 11px;
  line-height: 1.4;
`

const ErrorBanner = styled(WarningBanner)`
  border-color: ${palette.red};
  border-left-color: ${palette.red};
  background-color: rgba(207, 81, 73, 0.14);
  color: #f2aca7;
`

const BannerIcon = styled.span`
  font-size: 14px;
  line-height: 1.2;
`

const BannerTitle = styled.div`
  font-weight: bold;
  margin-bottom: 3px;
`

/* The session state is categorical rather than telemetry, so it uses the UI font instead of the
   shared StateValue's monospace one. */
const SessionStateText = styled(StateValue)`
  font-family: inherit;
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
  const { runtimeParametersValid, flagMissingRuntimeParameters } = useContext(ModuleListContext)
  const { recordingsList } = useContext(RecordingContext)
  const { datasetList } = useContext(EegSimulatorContext)
  const { clearAllLogs } = useContext(LogContext)
  const { diskStatus, diskSeverity } = useDiskStatus()
  const [displayedState, setDisplayedState] = useState(sessionState.state)
  const [startError, setStartError] = useState<string | null>(null)

  /* Add 500ms hysteresis to prevent rapid flashing of state changes,
     as states (like INITIALIZING, FINALIZING) may sometimes change very quickly. */
  useEffect(() => {
    const timeoutId = setTimeout(() => {
      setDisplayedState(sessionState.state)
    }, 500)

    return () => clearTimeout(timeoutId)
  }, [sessionState.state])

  const handleStartSession = () => {
    setStartError(null)

    /* Preflight: re-check the disk status at the moment Start is pressed, since it is updated
       periodically and may have changed since the button was rendered. Refuse to start rather
       than letting the session enter the running state and die when recording fails. */
    if (getDiskSeverity(diskStatus) === 'error') {
      return
    }

    /* Every runtime parameter is required. Rather than disabling the button, point out
       the ones that are still unset so that the user can see what is blocking the start. */
    if (!runtimeParametersValid) {
      flagMissingRuntimeParameters()
      return
    }

    // Clear pipeline logs before starting the session
    clearAllLogs()

    startSession((success: boolean, message?: string) => {
      if (success) {
        console.log('Session start requested successfully')
      } else {
        console.log('Failed to start session:', message)
        setStartError(message || 'The session could not be started.')
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
      return dataSource === 'recording' ? 'Replay' : 'Start'
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

  const isRunning = sessionState.state === SessionStateValue.RUNNING

  /* Free disk space below the error threshold blocks starting a session, but must never block
     stopping one that is already running. */
  const isBlockedByDiskSpace = !isRunning && diskSeverity === 'error'

  const isButtonDisabled = isNoDataAvailable ||
    isBlockedByDiskSpace ||
    sessionState.state === SessionStateValue.INITIALIZING ||
    sessionState.state === SessionStateValue.FINALIZING

  const ButtonComponent = isRunning ? StyledRedButton : StyledButton
  return (
    <Container>
      <ConfigTitle>Session</ConfigTitle>

      <div>
        {diskStatus && diskSeverity === 'error' && (
          <ErrorBanner>
            <BannerIcon>{'\u26D4'}</BannerIcon>
            <div>
              <BannerTitle>
                {isRunning ? 'Critically low disk space' : 'Cannot start session'}
              </BannerTitle>
              <div>
                Only {formatGiB(diskStatus.free_bytes)} GiB free; at least{' '}
                {formatGiB(diskStatus.error_threshold_bytes)} GiB is required.
              </div>
              <div>
                {isRunning
                  ? 'Free disk space; recording may fail.'
                  : 'Free disk space and try again.'}
              </div>
            </div>
          </ErrorBanner>
        )}

        {diskStatus && diskSeverity === 'warning' && (
          <WarningBanner>
            <BannerIcon>{'\u26A0'}</BannerIcon>
            <div>
              <BannerTitle>
                Low disk space &mdash; {formatGiB(diskStatus.free_bytes)} GiB remaining
              </BannerTitle>
              <div>
                Experiments may run out of space and fail partway through. Free disk space when convenient.
              </div>
            </div>
          </WarningBanner>
        )}

        {startError && (
          <ErrorBanner>
            <BannerIcon>{'\u26D4'}</BannerIcon>
            <div>
              <BannerTitle>Cannot start session</BannerTitle>
              <div>{startError}</div>
            </div>
          </ErrorBanner>
        )}
      </div>

      <StateRow style={{ marginBottom: 8 }}>
        <StateTitle>Control:</StateTitle>
        <ButtonComponent
          onClick={handleButtonClick}
          disabled={isButtonDisabled}
        >
          {getButtonText()}
        </ButtonComponent>
      </StateRow>

      <StateRow>
        <StateTitle>State:</StateTitle>
        <SessionStateText>{getStateDisplayText(displayedState)}</SessionStateText>
      </StateRow>
    </Container>
  )
}