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

export const StimulationDisplay: React.FC = () => {
  const { loopbackLatency, pulseProcessingLatency, eventProcessingLatency, decisionTrace, attemptTrace, setPulseProcessingLatency, setEventProcessingLatency } = useContext(SessionStatisticsContext)
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

  // Decision latency comes from the live decision trace stream
  const formattedLoopbackLatency = loopbackLatency ? (loopbackLatency.latency * 1000).toFixed(1) + ' ms' : '\u2013'
  const formattedPulseProcessingLatency = pulseProcessingLatency ? (pulseProcessingLatency.latency).toFixed(1) + ' s' : '\u2013'
  const formattedEventProcessingLatency = eventProcessingLatency ? (eventProcessingLatency.latency * 1000).toFixed(1) + ' ms' : '\u2013'
  const formattedDecisionPathLatency = decisionTrace?.decision_path_latency
    ? (decisionTrace.decision_path_latency * 1000).toFixed(1) + ' ms'
    : '\u2013'
  const formattedPreprocessorDuration = decisionTrace?.preprocessor_duration
    ? (decisionTrace.preprocessor_duration * 1000).toFixed(1) + ' ms'
    : '\u2013'
  const formattedDeciderDuration = decisionTrace?.decider_duration
    ? (decisionTrace.decider_duration * 1000).toFixed(1) + ' ms'
    : '\u2013'

  // Pulse info comes from the attempt trace (with embedded decision for reference time)
  const trialDecision = latestAttemptTrace?.decision
  const isReactiveMode =
    latestAttemptTrace?.requested_stimulation_time !== undefined &&
    trialDecision?.reference_sample_time !== undefined &&
    Math.abs(latestAttemptTrace.requested_stimulation_time - trialDecision.reference_sample_time) <= 0.001

  const formattedReferenceSampleTime = trialDecision?.reference_sample_time
    ? trialDecision.reference_sample_time.toFixed(3).replace(/\.?0+$/, '') + ' s'
    : '\u2013'
  const formattedRequestedStimulationOffset =
    latestAttemptTrace?.requested_stimulation_time !== undefined && trialDecision?.reference_sample_time !== undefined
    ? '+' + ((latestAttemptTrace.requested_stimulation_time - trialDecision.reference_sample_time) * 1000).toFixed(1) + ' ms'
    : '\u2013'
  const stimulationHorizonMs =
    latestAttemptTrace?.stimulation_horizon !== undefined ? latestAttemptTrace.stimulation_horizon * 1000 : undefined
  const formattedStimulationHorizon =
    stimulationHorizonMs !== undefined ? '>' + stimulationHorizonMs.toFixed(1) + ' ms' : '\u2013'
  const formattedStrictHorizon = latestAttemptTrace?.strict_stimulation_horizon !== undefined
    ? '>' + (latestAttemptTrace.strict_stimulation_horizon * 1000).toFixed(1) + ' ms'
    : '\u2013'
  const formattedTimingOffset = latestAttemptTrace?.timing_offset !== undefined && latestAttemptTrace.timing_offset !== 0
    ? (latestAttemptTrace.timing_offset * 1000).toFixed(1) + ' ms'
    : '\u2013'
  const baseStatus = latestAttemptTrace?.status !== undefined ? getStatusLabel(latestAttemptTrace.status) : '\u2013'
  const reactiveTooLateStatus =
    isReactiveMode && latestAttemptTrace?.status === 8 && stimulationHorizonMs !== undefined
      ? `Too late (+${stimulationHorizonMs.toFixed(1)} ms)`
      : null
  const formattedStatus = reactiveTooLateStatus ?? baseStatus

  return (
    <>
      <StimulationPanelTitle>Stimulation</StimulationPanelTitle>
      <StimulationPanel>
        {/* Latency */}
        <StateRow>
          <StateTitle>Latency</StateTitle>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Loopback</IndentedStateTitle>
          <StateValue>{formattedLoopbackLatency}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Pulse processing</IndentedStateTitle>
          <StateValue>{formattedPulseProcessingLatency}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Event processing</IndentedStateTitle>
          <StateValue>{formattedEventProcessingLatency}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Decision path</IndentedStateTitle>
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
        <br />
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
        <div style={{ height: '8px' }} />
        {!isReactiveMode && (
          <>
            <StateRow>
              <IndentedStateTitle>Horizon</IndentedStateTitle>
              <StateValue>{formattedStimulationHorizon}</StateValue>
            </StateRow>
            <StateRow>
              <DoubleIndentedStateTitle>Strict</DoubleIndentedStateTitle>
              <StateValue>{formattedStrictHorizon}</StateValue>
            </StateRow>
          </>
        )}
      </StimulationPanel>
    </>
  )
}
