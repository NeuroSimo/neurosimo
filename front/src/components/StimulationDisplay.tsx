import React, { useContext, useState, useEffect } from 'react'
import styled from 'styled-components'

import {
  DoubleIndentedStateTitle,
  IndentedStateTitle,
  StyledPanel,
  StateRow,
  StateTitle,
  StateValue,
  DASHBOARD_PANEL_OFFSET_FROM_TOP,
  DASHBOARD_PANEL_HEIGHT,
} from 'styles/General'

import { SessionStatisticsContext, getStatusLabel, AttemptTrace } from 'providers/SessionStatisticsProvider'
import { useSession, SessionStateValue } from 'providers/SessionProvider'

const StimulationPanelTitle = styled.div`
  width: 220px;
  position: fixed;
  top: ${DASHBOARD_PANEL_OFFSET_FROM_TOP}px;
  right: 24px;
  z-index: 1001;
  text-align: left;
  font-size: 12px;
  font-weight: bold;
`

const StimulationPanel = styled(StyledPanel)`
  width: 220px;
  height: ${DASHBOARD_PANEL_HEIGHT}px;
  position: fixed;
  top: ${DASHBOARD_PANEL_OFFSET_FROM_TOP + 20}px;
  right: 4px;
  z-index: 1000;
`

const SectionSpacer = styled.div<{ $height?: number }>`
  height: ${props => props.$height ?? 6}px;
`

export const StimulationDisplay: React.FC = () => {
  const { pulseProcessingLatency, eventProcessingLatency, decisionTrace, attemptTrace, setPulseProcessingLatency, setEventProcessingLatency } = useContext(SessionStatisticsContext)
  const { sessionState } = useSession()
  const [latestAttemptTrace, setLatestAttemptTrace] = useState<AttemptTrace | null>(null)
  const [previousSessionState, setPreviousSessionState] = useState<SessionStateValue>(SessionStateValue.STOPPED)

  // Reset traces when session starts
  useEffect(() => {
    if (previousSessionState === SessionStateValue.STOPPED &&
        (sessionState.state === SessionStateValue.INITIALIZING || sessionState.state === SessionStateValue.RUNNING)) {
      setLatestAttemptTrace(null)
      setPulseProcessingLatency(null)
      setEventProcessingLatency(null)
    }
    setPreviousSessionState(sessionState.state)
  }, [sessionState.state, previousSessionState, setPulseProcessingLatency, setEventProcessingLatency])

  // Update latest attempt trace from context
  useEffect(() => {
    if (attemptTrace) {
      setLatestAttemptTrace(attemptTrace)
    }
  }, [attemptTrace])

  // Decision trace
  const decisionPathLatency = decisionTrace
    ? decisionTrace.preprocessor_duration + decisionTrace.decider_duration + decisionTrace.overhead_duration
    : undefined
  const formattedDecisionPathLatency = decisionPathLatency
    ? (decisionPathLatency * 1000).toFixed(1) + ' ms'
    : '\u2013'
  const formattedEegDeviceProcessingDuration = decisionTrace?.eeg_device_processing_duration
    ? (decisionTrace.eeg_device_processing_duration * 1000).toFixed(1) + ' ms'
    : '\u2013'
  const formattedPreprocessorDuration = decisionTrace?.preprocessor_duration
    ? (decisionTrace.preprocessor_duration * 1000).toFixed(1) + ' ms'
    : '\u2013'
  const formattedDeciderDuration = decisionTrace?.decider_duration
    ? (decisionTrace.decider_duration * 1000).toFixed(1) + ' ms'
    : '\u2013'
  const formattedOverheadDuration = decisionTrace?.overhead_duration
    ? (decisionTrace.overhead_duration * 1000).toFixed(1) + ' ms'
    : '\u2013'

  const formattedPulseProcessingLatency = pulseProcessingLatency ? (pulseProcessingLatency.latency).toFixed(1) + ' s' : '\u2013'
  const formattedEventProcessingLatency = eventProcessingLatency ? (eventProcessingLatency.latency * 1000).toFixed(1) + ' ms' : '\u2013'

  const referenceTime = latestAttemptTrace?.reference_time

  const formattedReferenceSampleTime = referenceTime !== undefined
    ? referenceTime.toFixed(3).replace(/\.?0+$/, '') + ' s'
    : '\u2013'
  const formattedRequestedStimulationOffset =
    latestAttemptTrace?.requested_stimulation_time !== undefined && referenceTime !== undefined
    ? '+' + ((latestAttemptTrace.requested_stimulation_time - referenceTime) * 1000).toFixed(1) + ' ms'
    : '\u2013'
  const formattedTimingOffset = latestAttemptTrace?.timing_offset !== undefined && latestAttemptTrace.timing_offset !== 0
    ? (latestAttemptTrace.timing_offset * 1000).toFixed(1) + ' ms'
    : '\u2013'
  const formattedStatus = latestAttemptTrace?.status !== undefined ? getStatusLabel(latestAttemptTrace.status) : '\u2013'

  return (
    <>
      <StimulationPanelTitle>Stimulation</StimulationPanelTitle>
      <StimulationPanel>
        {/* Latency */}
        <StateRow>
          <StateTitle>Decision</StateTitle>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>EEG device</IndentedStateTitle>
          <StateValue>{formattedEegDeviceProcessingDuration}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Time to decision</IndentedStateTitle>
          <StateValue>{formattedDecisionPathLatency}</StateValue>
        </StateRow>
        <StateRow>
          <DoubleIndentedStateTitle>Preprocessor</DoubleIndentedStateTitle>
          <StateValue>{formattedPreprocessorDuration}</StateValue>
        </StateRow>
        <StateRow>
          <DoubleIndentedStateTitle>Decider</DoubleIndentedStateTitle>
          <StateValue>{formattedDeciderDuration}</StateValue>
        </StateRow>
        <StateRow>
          <DoubleIndentedStateTitle>Overhead</DoubleIndentedStateTitle>
          <StateValue>{formattedOverheadDuration}</StateValue>
        </StateRow>
        <SectionSpacer />
        <StateRow>
          <IndentedStateTitle>Pulse processing</IndentedStateTitle>
          <StateValue>{formattedPulseProcessingLatency}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Event processing</IndentedStateTitle>
          <StateValue>{formattedEventProcessingLatency}</StateValue>
        </StateRow>
        <SectionSpacer />
        {/* Pulse */}
        <StateRow>
          <StateTitle>Pulse</StateTitle>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Decided at</IndentedStateTitle>
          <StateValue>{formattedReferenceSampleTime}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Requested for</IndentedStateTitle>
          <StateValue>{formattedRequestedStimulationOffset}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Timing offset</IndentedStateTitle>
          <StateValue>{formattedTimingOffset}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Status</IndentedStateTitle>
          <StateValue>{formattedStatus}</StateValue>
        </StateRow>
      </StimulationPanel>
    </>
  )
}
