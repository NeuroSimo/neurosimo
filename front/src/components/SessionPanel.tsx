import React, { useState, useEffect, useLayoutEffect, useRef, useContext } from 'react'
import styled from 'styled-components'

import { ConfigPanel, ConfigTitle, CONFIG_PANEL_WIDTH, StateRow, StateTitle, StateValue, StyledButton, StyledRedButton } from 'styles/General'
import { useSession, SessionStateValue } from 'providers/SessionProvider'
import { useSessionConfig } from 'providers/SessionConfigProvider'
import { ModuleListContext } from 'providers/ModuleListProvider'
import { RecordingContext } from 'providers/RecordingProvider'
import { EegSimulatorContext } from 'providers/EegSimulatorProvider'
import { LogContext } from 'providers/LogProvider'
import { useDiskStatus, getDiskSeverity, formatGiB } from 'providers/DiskStatusProvider'

const Container = styled(ConfigPanel)`
  width: ${CONFIG_PANEL_WIDTH}px;
  position: relative;
  margin-top: 0;
  margin-left: 0;
  left: 0;
`

/* Deliberately loud: a low-disk condition must not be mistakable for ordinary status text. */
const WarningBanner = styled.div`
  display: flex;
  align-items: flex-start;
  gap: 8px;
  margin: 0 9px 12px 0;
  padding: 8px 10px;
  border: 1px solid #e0a800;
  border-left: 5px solid #e0a800;
  border-radius: 3px;
  background-color: #fff6d9;
  color: #5c4600;
  font-size: 11px;
  line-height: 1.4;
`

const ErrorBanner = styled(WarningBanner)`
  border-color: #b00020;
  border-left-color: #b00020;
  background-color: #fdecee;
  color: #7a0016;
`

const BannerIcon = styled.span`
  font-size: 14px;
  line-height: 1.2;
`

const BannerTitle = styled.div`
  font-weight: bold;
  margin-bottom: 3px;
`

/* The panel sits at a fixed offset from the top of the pipeline column, so growing downwards
   would push it outside its designated area. Instead, the panel is shifted up by the height of
   its banners, which keeps its bottom edge in place and makes it extend upwards. */
const Banners = styled.div``

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
  const bannersRef = useRef<HTMLDivElement>(null)
  const [bannerHeight, setBannerHeight] = useState(0)

  useLayoutEffect(() => {
    const element = bannersRef.current
    if (!element) return

    const observer = new ResizeObserver(() => setBannerHeight(element.offsetHeight))
    observer.observe(element)
    setBannerHeight(element.offsetHeight)

    return () => observer.disconnect()
  }, [])

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

  const stateText = isBlockedByDiskSpace && displayedState === SessionStateValue.STOPPED
    ? 'Blocked'
    : getStateDisplayText(displayedState)

  const ButtonComponent = isRunning ? StyledRedButton : StyledButton
  return (
    <Container style={{ marginTop: -bannerHeight }}>
      <ConfigTitle>Session</ConfigTitle>

      <Banners ref={bannersRef}>
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
                Experiments may run out of space and fail partway through. Free disk space before
                continuing.
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
      </Banners>

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
        <StateValue>{stateText}</StateValue>
      </StateRow>
    </Container>
  )
}