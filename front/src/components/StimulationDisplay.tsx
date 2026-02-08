import React, { useContext } from 'react'
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

import { PipelineContext } from 'providers/PipelineProvider'

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
  const { loopbackLatency, decisionTrace } = useContext(PipelineContext)

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
  const formattedReferenceSampleTime = decisionTrace?.reference_sample_time
    ? decisionTrace.reference_sample_time.toFixed(3).replace(/\.?0+$/, '') + ' s'
    : '\u2013'
  const formattedRequestedStimulationOffset = 
    decisionTrace?.requested_stimulation_time !== undefined && decisionTrace?.reference_sample_time !== undefined
    ? '+' + ((decisionTrace.requested_stimulation_time - decisionTrace.reference_sample_time) * 1000).toFixed(1) + ' ms'
    : '\u2013'
  const formattedStimulationHorizon = decisionTrace?.stimulation_horizon
    ? '>' + (decisionTrace.stimulation_horizon * 1000).toFixed(1) + ' ms'
    : '\u2013'
  const formattedTimingError = decisionTrace?.timing_error !== undefined
    ? (decisionTrace.timing_error * 1000).toFixed(1) + ' ms'
    : '\u2013'

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
        <br />
        <StateRow>
          <IndentedStateTitle>Horizon</IndentedStateTitle>
          <StateValue>{formattedStimulationHorizon}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Timing error</IndentedStateTitle>
          <StateValue>{formattedTimingError}</StateValue>
        </StateRow>
      </StimulationPanel>
    </>
  )
}
