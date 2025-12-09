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
import { SessionContext, SessionState } from 'providers/SessionProvider'

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
  const { timingLatency, setTimingLatency } = useContext(PipelineContext)
  const { timingError, setTimingError } = useContext(PipelineContext)
  const { decisionInfo } = useContext(PipelineContext)

  const { session } = useContext(SessionContext)

  const sessionState = session?.state

  const [positiveDecision, setPositiveDecision] = useState<any>(null)
  const [latestDecision, setLatestDecision] = useState<any>(null)

  useEffect(() => {
    if (decisionInfo) {
      // Update the latest decision
      setLatestDecision(decisionInfo)

      // Update positive decision if `stimulate` is true
      if (decisionInfo.stimulate) {
        setPositiveDecision(decisionInfo)
      }
    }
  }, [decisionInfo])

  useEffect(() => {
    if (sessionState?.value === SessionState.STOPPED) {
      setTimingLatency(null)
      setTimingError(null)
      setPositiveDecision(null)
      setLatestDecision(null)
    }
  }, [sessionState])

  const formattedLatency =
    timingLatency && sessionState?.value === SessionState.STARTED
      ? (timingLatency.latency * 1000).toFixed(1) + ' ms'
      : '\u2013'

  const formattedError =
    timingError && sessionState?.value === SessionState.STARTED
      ? (timingError.error * 1000).toFixed(1) + ' ms'
      : '\u2013'

  // Latest Decision Stats
  const formattedLatestDecisionTime = latestDecision?.decision_time
    ? latestDecision.decision_time.toFixed(1) + ' s'
    : '\u2013'

  const formattedLatestDeciderLatency = latestDecision?.decider_latency
    ? (latestDecision.decider_latency * 1000).toFixed(1) + ' ms'
    : '\u2013'

  const formattedLatestPreprocessorLatency = latestDecision?.preprocessor_latency
    ? (latestDecision.preprocessor_latency * 1000).toFixed(1) + ' ms'
    : '\u2013'

  const formattedLatestTotalLatency = latestDecision?.total_latency
    ? (latestDecision.total_latency * 1000).toFixed(1) + ' ms'
    : '\u2013'

  // Positive Decision Stats
  const formattedPositiveDecisionTime = positiveDecision?.decision_time
    ? positiveDecision.decision_time.toFixed(1) + ' s'
    : '\u2013'

  const formattedPositiveDeciderLatency = positiveDecision?.decider_latency
    ? (positiveDecision.decider_latency * 1000).toFixed(1) + ' ms'
    : '\u2013'

  const formattedPositivePreprocessorLatency = positiveDecision?.preprocessor_latency
    ? (positiveDecision.preprocessor_latency * 1000).toFixed(1) + ' ms'
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
        <StateValue>{formattedLatestDecisionTime}</StateValue>
      </StateRow>
      <StateRow>
        <IndentedStateTitle>Latency</IndentedStateTitle>
      </StateRow>
      <StateRow>
        <DoubleIndentedStateTitle>Decider</DoubleIndentedStateTitle>
        <StateValue>{formattedLatestDeciderLatency}</StateValue>
      </StateRow>
      <StateRow>
        <DoubleIndentedStateTitle>Preprocessor</DoubleIndentedStateTitle>
        <StateValue>{formattedLatestPreprocessorLatency}</StateValue>
      </StateRow>
      <StateRow>
        <DoubleIndentedStateTitle>Total</DoubleIndentedStateTitle>
        <StateValue>{formattedLatestTotalLatency}</StateValue>
      </StateRow>
      <br />
      {/* Positive Decision Info */}
      <StateRow>
        <IndentedStateTitle>Latest stimulation time</IndentedStateTitle>
        <StateValue>{formattedPositiveDecisionTime}</StateValue>
      </StateRow>
      <StateRow>
        <DoubleIndentedStateTitle>Decider</DoubleIndentedStateTitle>
        <StateValue>{formattedPositiveDeciderLatency}</StateValue>
      </StateRow>
      <StateRow>
        <DoubleIndentedStateTitle>Preprocessor</DoubleIndentedStateTitle>
        <StateValue>{formattedPositivePreprocessorLatency}</StateValue>
      </StateRow>
      <StateRow>
        <DoubleIndentedStateTitle>Total</DoubleIndentedStateTitle>
        <StateValue>{formattedPositiveTotalLatency}</StateValue>
      </StateRow>
      <StateRow>
        <DoubleIndentedStateTitle>Feasible</DoubleIndentedStateTitle>
        <StateValue>{positiveDecision?.feasible ? '\u2714' : '\u2718'}</StateValue>
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
