import React, { useContext } from 'react'
import styled from 'styled-components'

import { StyledPanel, StateRow, StateTitle, IndentedStateTitle, StateValue } from 'styles/General'

import { EegContext } from 'providers/EegProvider'

const EegStatisticsPanelTitle = styled.div`
  width: 340px;
  position: fixed;
  top: 896px;
  right: 5px;
  z-index: 1001;
  text-align: left;
  font-size: 20px;
  font-weight: bold;
`

const EegStatisticsPanel = styled(StyledPanel)`
  width: 300px;
  height: 280px;
  position: fixed;
  top: 928px;
  right: 5px;
  z-index: 1000;
`

export const EegStatisticsDisplay: React.FC = () => {
  const { eegStatistics, droppedSamples } = useContext(EegContext)

  const formatValue = (value: number | undefined, formatter: (value: number) => string): string => {
    if (value === undefined || value === null || value === 0) {
      return '\u2013'
    }
    return formatter(value)
  }

  const formatTimeToMicroseconds = (timeInSeconds?: number): string =>
    formatValue(timeInSeconds, (time) => `${(time * 1_000_000).toFixed(0)} µs`)

  const formatTimeToMilliseconds = (timeInSeconds?: number, precision = 1): string =>
    formatValue(timeInSeconds, (time) => `${(time * 1_000).toFixed(precision)} ms`)

  return (
    <>
      <EegStatisticsPanelTitle>Statistics</EegStatisticsPanelTitle>
      <EegStatisticsPanel>
        <StateRow>
          <StateTitle>Samples:</StateTitle>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Raw:</IndentedStateTitle>
          <StateValue>{eegStatistics?.num_of_raw_samples ?? '\u2013'}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Preprocessed:</IndentedStateTitle>
          <StateValue>{eegStatistics?.num_of_preprocessed_samples ?? '\u2013'}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Dropped:</IndentedStateTitle>
          <StateValue>{droppedSamples !== null && droppedSamples > 0 ? droppedSamples : '\u2013'}</StateValue>
        </StateRow>
        <br />
        <StateRow>
          <StateTitle>Preprocessing time:</StateTitle>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Median</IndentedStateTitle>
          <StateValue>{formatTimeToMicroseconds(eegStatistics?.preprocessing_time_median)}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Q95</IndentedStateTitle>
          <StateValue>{formatTimeToMicroseconds(eegStatistics?.preprocessing_time_q95)}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Max</IndentedStateTitle>
          <StateValue>{formatTimeToMicroseconds(eegStatistics?.preprocessing_time_max)}</StateValue>
        </StateRow>
        <br />
        <StateRow>
          <StateTitle>Maximum sample interval:</StateTitle>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Raw</IndentedStateTitle>
          <StateValue>{formatTimeToMilliseconds(eegStatistics?.max_time_between_raw_samples)}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>Preprocessed</IndentedStateTitle>
          <StateValue>{formatTimeToMilliseconds(eegStatistics?.max_time_between_preprocessed_samples)}</StateValue>
        </StateRow>
      </EegStatisticsPanel>
    </>
  )
}
