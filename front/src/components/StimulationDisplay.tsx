import React, { useContext, useEffect, useState } from 'react'
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

const statusLabels = {
  0: 'No decision',
  1: 'Decided yes',
  2: 'Scheduled',
  3: 'Rejected',
  4: 'Fired',
  5: 'Pulse observed',
  6: 'Missed',
  7: 'Error'
}

export const StimulationDisplay: React.FC = () => {
  const { pipelineLatency, setPipelineLatency } = useContext(PipelineContext)
  const { decisionTrace } = useContext(PipelineContext)

  const formattedLatency = pipelineLatency ? (pipelineLatency.latency * 1000).toFixed(1) + ' ms' : '\u2013'

  // Decision Stats
  const formattedStatus = decisionTrace ? statusLabels[decisionTrace.status as keyof typeof statusLabels] || 'Unknown' : '\u2013'

  const formattedReferenceSampleTime = decisionTrace?.reference_sample_time
    ? decisionTrace.reference_sample_time.toFixed(1) + ' s'
    : '\u2013'

  const formattedDeciderDuration = decisionTrace?.decider_duration
    ? (decisionTrace.decider_duration * 1000).toFixed(1) + ' ms'
    : '\u2013'

  const formattedPreprocessorDuration = decisionTrace?.preprocessor_duration
    ? (decisionTrace.preprocessor_duration * 1000).toFixed(1) + ' ms'
    : '\u2013'

  const formattedRequestedStimulationTime = decisionTrace?.requested_stimulation_time
    ? decisionTrace.requested_stimulation_time.toFixed(1) + ' s'
    : '\u2013'

  const formattedActualStimulationTime = decisionTrace?.actual_stimulation_time
    ? decisionTrace.actual_stimulation_time.toFixed(1) + ' s'
    : '\u2013'

  const formattedTimingError = decisionTrace?.timing_error !== undefined
    ? (decisionTrace.timing_error * 1000).toFixed(1) + ' ms'
    : '\u2013'

  const formattedPulseConfirmed = decisionTrace ? (decisionTrace.pulse_confirmed ? 'Yes' : 'No') : '\u2013'

  return (
    <>
      <StimulationPanelTitle>Stimulation</StimulationPanelTitle>
      <StimulationPanel>
        {/* Decision trace */}
        <StateRow>
          <StateTitle>Status:</StateTitle>
          <StateValue>{formattedStatus}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Stimulate</IndentedStateTitle>
          <StateValue>{decisionTrace ? (decisionTrace.stimulate ? 'Yes' : 'No') : '\u2013'}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Reference time</IndentedStateTitle>
          <StateValue>{formattedReferenceSampleTime}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Requested time</IndentedStateTitle>
          <StateValue>{formattedRequestedStimulationTime}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Actual time</IndentedStateTitle>
          <StateValue>{formattedActualStimulationTime}</StateValue>
        </StateRow>
        <br />
        {/* Processing Times */}
        <StateRow>
          <StateTitle>Processing:</StateTitle>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Decider</IndentedStateTitle>
          <StateValue>{formattedDeciderDuration}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Preprocessor</IndentedStateTitle>
          <StateValue>{formattedPreprocessorDuration}</StateValue>
        </StateRow>
        <br />
        {/* Pulse Info */}
        <StateRow>
          <StateTitle>Pulse:</StateTitle>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Confirmed</IndentedStateTitle>
          <StateValue>{formattedPulseConfirmed}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Timing error</IndentedStateTitle>
          <StateValue>{formattedTimingError}</StateValue>
        </StateRow>
        <br />
        {/* Timing Info */}
        <StateRow>
          <StateTitle>Timing:</StateTitle>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Latency</IndentedStateTitle>
          <StateValue>{formattedLatency}</StateValue>
        </StateRow>
      </StimulationPanel>
    </>
  )
}
