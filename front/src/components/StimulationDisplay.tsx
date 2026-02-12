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

import { SessionStatisticsContext, getStatusLabel } from 'providers/SessionStatisticsProvider'

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
  const { loopbackLatency, decisionTrace } = useContext(SessionStatisticsContext)
  const [positiveDecisionTrace, setPositiveDecisionTrace] = useState<any>(null)

  // Track the latest decision trace (not "Decided no")
  useEffect(() => {
    if (decisionTrace && decisionTrace.status !== 1) {
      setPositiveDecisionTrace(decisionTrace)
    }
  }, [decisionTrace])

  // Latency
  const formattedLoopbackLatency = loopbackLatency ? (loopbackLatency.latency * 1000).toFixed(1) + ' ms' : '\u2013'
  const formattedDecisionPathLatency = decisionTrace?.decision_path_latency
    ? (decisionTrace.decision_path_latency * 1000).toFixed(1) + ' ms'
    : '\u2013'
  const formattedPreprocessorDuration = decisionTrace?.preprocessor_duration
    ? (decisionTrace.preprocessor_duration * 1000).toFixed(1) + ' ms'
    : '\u2013'
  const formattedDeciderDuration = decisionTrace?.decider_duration
    ? (decisionTrace.decider_duration * 1000).toFixed(1) + ' ms'
    : '\u2013'

  // Pulse
  const isReactiveMode =
    positiveDecisionTrace?.requested_stimulation_time !== undefined &&
    positiveDecisionTrace?.reference_sample_time !== undefined &&
    Math.abs(positiveDecisionTrace.requested_stimulation_time - positiveDecisionTrace.reference_sample_time) <= 0.001

  const formattedReferenceSampleTime = positiveDecisionTrace?.reference_sample_time
    ? positiveDecisionTrace.reference_sample_time.toFixed(3).replace(/\.?0+$/, '') + ' s'
    : '\u2013'
  const formattedRequestedStimulationOffset =
    positiveDecisionTrace?.requested_stimulation_time !== undefined && positiveDecisionTrace?.reference_sample_time !== undefined
    ? '+' + ((positiveDecisionTrace.requested_stimulation_time - positiveDecisionTrace.reference_sample_time) * 1000).toFixed(1) + ' ms'
    : '\u2013'
  const stimulationHorizonMs =
    positiveDecisionTrace?.stimulation_horizon !== undefined ? positiveDecisionTrace.stimulation_horizon * 1000 : undefined
  const formattedStimulationHorizon =
    stimulationHorizonMs !== undefined ? '>' + stimulationHorizonMs.toFixed(1) + ' ms' : '\u2013'
  const formattedStrictHorizon = positiveDecisionTrace?.strict_stimulation_horizon !== undefined
    ? '>' + (positiveDecisionTrace.strict_stimulation_horizon * 1000).toFixed(1) + ' ms'
    : '\u2013'
  const formattedTimingOffset = positiveDecisionTrace?.timing_offset !== undefined && positiveDecisionTrace.timing_offset !== 0
    ? (positiveDecisionTrace.timing_offset * 1000).toFixed(1) + ' ms'
    : '\u2013'
  const baseStatus = positiveDecisionTrace?.status !== undefined ? getStatusLabel(positiveDecisionTrace.status) : '\u2013'
  const reactiveTooLateStatus =
    isReactiveMode && positiveDecisionTrace?.status === 8 && stimulationHorizonMs !== undefined
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
