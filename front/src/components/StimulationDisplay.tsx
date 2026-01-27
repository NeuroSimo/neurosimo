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

export const StimulationDisplay: React.FC = () => {
  const { pipelineLatency, setPipelineLatency } = useContext(PipelineContext)
  const { timingError, setTimingError } = useContext(PipelineContext)
  const { decisionTrace } = useContext(PipelineContext)

  const [positiveDecision, setPositiveDecision] = useState<any>(null)
  const [latestDecision, setLatestDecision] = useState<any>(null)

  useEffect(() => {
    if (decisionTrace) {
      // Update the latest decision
      setLatestDecision(decisionTrace)

      // Update positive decision if `stimulate` is true
      if (decisionTrace.stimulate) {
        setPositiveDecision(decisionTrace)
      }
    }
  }, [decisionTrace])

  const formattedLatency = pipelineLatency ? (pipelineLatency.latency * 1000).toFixed(1) + ' ms' : '\u2013'

  const formattedError = timingError ? (timingError.error * 1000).toFixed(1) + ' ms' : '\u2013'

  // Latest Decision Stats
  const formattedLatestSampleTime = latestDecision?.sample_time
    ? latestDecision.sample_time.toFixed(1) + ' s'
    : '\u2013'

  const formattedLatestDeciderProcessingDuration = latestDecision?.decider_processing_duration
    ? (latestDecision.decider_processing_duration * 1000).toFixed(1) + ' ms'
    : '\u2013'

  const formattedLatestPreprocessorProcessingDuration = latestDecision?.preprocessor_processing_duration
    ? (latestDecision.preprocessor_processing_duration * 1000).toFixed(1) + ' ms'
    : '\u2013'

  const formattedLatestTotalLatency = latestDecision?.total_latency
    ? (latestDecision.total_latency * 1000).toFixed(1) + ' ms'
    : '\u2013'

  // Positive Decision Stats
  const formattedPositiveSampleTime = positiveDecision?.sample_time
    ? positiveDecision.sample_time.toFixed(1) + ' s'
    : '\u2013'

  const formattedPositiveDeciderProcessingDuration = positiveDecision?.decider_processing_duration
    ? (positiveDecision.decider_processing_duration * 1000).toFixed(1) + ' ms'
    : '\u2013'

  const formattedPositivePreprocessorProcessingDuration = positiveDecision?.preprocessor_processing_duration
    ? (positiveDecision.preprocessor_processing_duration * 1000).toFixed(1) + ' ms'
    : '\u2013'

  const formattedPositiveTotalLatency = positiveDecision?.total_latency
    ? (positiveDecision.total_latency * 1000).toFixed(1) + ' ms'
    : '\u2013'

  return (
    <>
      <StimulationPanelTitle>Stimulation</StimulationPanelTitle>
      <StimulationPanel>
        {/* Latest Decision Info */}
        <StateRow>
        <StateTitle>Decisions:</StateTitle>
      </StateRow>
      <StateRow>
        <IndentedStateTitle>Latest time</IndentedStateTitle>
        <StateValue>{formattedLatestSampleTime}</StateValue>
      </StateRow>
      <StateRow>
        <IndentedStateTitle>Latency</IndentedStateTitle>
      </StateRow>
      <StateRow>
        <DoubleIndentedStateTitle>Decider</DoubleIndentedStateTitle>
        <StateValue>{formattedLatestDeciderProcessingDuration}</StateValue>
      </StateRow>
      <StateRow>
        <DoubleIndentedStateTitle>Preprocessor</DoubleIndentedStateTitle>
        <StateValue>{formattedLatestPreprocessorProcessingDuration}</StateValue>
      </StateRow>
      <StateRow>
        <DoubleIndentedStateTitle>Total</DoubleIndentedStateTitle>
        <StateValue>{formattedLatestTotalLatency}</StateValue>
      </StateRow>
      <br />
      {/* Positive Decision Info */}
      <StateRow>
        <IndentedStateTitle>Latest stimulation time</IndentedStateTitle>
        <StateValue>{formattedPositiveSampleTime}</StateValue>
      </StateRow>
      <StateRow>
        <DoubleIndentedStateTitle>Decider</DoubleIndentedStateTitle>
        <StateValue>{formattedPositiveDeciderProcessingDuration}</StateValue>
      </StateRow>
      <StateRow>
        <DoubleIndentedStateTitle>Preprocessor</DoubleIndentedStateTitle>
        <StateValue>{formattedPositivePreprocessorProcessingDuration}</StateValue>
      </StateRow>
      <StateRow>
        <DoubleIndentedStateTitle>Total</DoubleIndentedStateTitle>
        <StateValue>{formattedPositiveTotalLatency}</StateValue>
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
      <StateRow>
        <IndentedStateTitle>Error</IndentedStateTitle>
        <StateValue>{formattedError}</StateValue>
      </StateRow>
      </StimulationPanel>
    </>
  )
}
