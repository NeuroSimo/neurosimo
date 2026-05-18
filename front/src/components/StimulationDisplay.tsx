import React, { useContext, useState, useEffect } from 'react'
import styled from 'styled-components'

import { ExperimentContext } from 'providers/ExperimentProvider'
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

const BreakdownDivider = styled.div`
  margin: 10px 12px 10px 12px;
  border-top: 1px solid #d8d8d8;
`

export const StimulationDisplay: React.FC = () => {
  const { pulseProcessingLatency, eventProcessingLatency, decisionTrace, attemptTrace, setPulseProcessingLatency, setEventProcessingLatency } = useContext(SessionStatisticsContext)

  const { sessionState } = useSession()
  const { experimentState } = useContext(ExperimentContext)

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
  const formattedTotalDuration = decisionTrace?.total_duration
    ? (decisionTrace.total_duration * 1000).toFixed(1) + ' ms'
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

  const formattedPulseProcessingTime = pulseProcessingLatency ? (pulseProcessingLatency.latency).toFixed(1) + ' s' : '\u2013'
  const formattedEventProcessingTime = eventProcessingLatency ? (eventProcessingLatency.latency * 1000).toFixed(1) + ' ms' : '\u2013'

  const referenceTime = latestAttemptTrace?.reference_time

  const formattedReferenceSampleTime = referenceTime !== undefined
    ? referenceTime.toFixed(1) + ' s'
    : '\u2013'
  const requestedStimulationOffsetSeconds =
    latestAttemptTrace?.requested_stimulation_time !== undefined && referenceTime !== undefined
      ? latestAttemptTrace.requested_stimulation_time - referenceTime
      : undefined
  const formattedRequestedStimulationOffset = requestedStimulationOffsetSeconds !== undefined
    ? requestedStimulationOffsetSeconds * 1000 > 1000
      ? '+' + requestedStimulationOffsetSeconds.toFixed(1) + ' s'
      : '+' + (requestedStimulationOffsetSeconds * 1000).toFixed(1) + ' ms'
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
          <StateTitle>Decision latency</StateTitle>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>EEG acquisition</IndentedStateTitle>
          <StateValue>{experimentState?.ongoing ? formattedEegDeviceProcessingDuration : '\u2013'}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Preprocessor</IndentedStateTitle>
          <StateValue>{experimentState?.ongoing ? formattedPreprocessorDuration : '\u2013'}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Decider</IndentedStateTitle>
          <StateValue>{experimentState?.ongoing ? formattedDeciderDuration : '\u2013'}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Overhead</IndentedStateTitle>
          <StateValue>{experimentState?.ongoing ? formattedOverheadDuration : '\u2013'}</StateValue>
        </StateRow>
        <BreakdownDivider />
        <StateRow>
          <IndentedStateTitle>Total</IndentedStateTitle>
          <StateValue>{experimentState?.ongoing ? formattedTotalDuration : '\u2013'}</StateValue>
        </StateRow>
        <SectionSpacer $height={16} />
        {/* Pulse */}
        <StateRow>
          <StateTitle>Pulse</StateTitle>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Reference time</IndentedStateTitle>
          <StateValue>{experimentState?.ongoing ? formattedReferenceSampleTime : '\u2013'}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Requested offset</IndentedStateTitle>
          <StateValue>{experimentState?.ongoing ? formattedRequestedStimulationOffset : '\u2013'}</StateValue>
        </StateRow>
        <SectionSpacer />
        <StateRow>
          <IndentedStateTitle>Timing error</IndentedStateTitle>
          <StateValue>{experimentState?.ongoing ? formattedTimingOffset : '\u2013'}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Status</IndentedStateTitle>
          <StateValue>{experimentState?.ongoing ? formattedStatus : '\u2013'}</StateValue>
        </StateRow>
        <SectionSpacer $height={16} />
        <StateRow>
          <StateTitle>Processing time</StateTitle>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Pulse</IndentedStateTitle>
          <StateValue>{experimentState?.ongoing ? formattedPulseProcessingTime : '\u2013'}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Event</IndentedStateTitle>
          <StateValue>{experimentState?.ongoing ? formattedEventProcessingTime : '\u2013'}</StateValue>
        </StateRow>
      </StimulationPanel>
    </>
  )
}
