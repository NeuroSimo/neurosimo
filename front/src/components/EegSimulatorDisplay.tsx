import React, { useContext, useEffect, useState } from 'react'
import styled from 'styled-components'

import { ToggleSwitch } from 'components/ToggleSwitch'
import { ValidatedInput } from 'components/ValidatedInput'

import {
  StyledPanel,
  Select,
  SmallerTitle,
  ConfigRow,
  ConfigLabel,
  ConfigValue,
  CONFIG_PANEL_WIDTH,
  StyledButton,
  StyledRedButton,
} from 'styles/General'

import { EegSimulatorContext, StreamerStateValue } from 'providers/EegSimulatorProvider'
import { PipelineContext } from 'providers/PipelineProvider'
import { setDatasetRos, setStartTimeRos } from 'ros/eeg_simulator'
import { formatTime, formatFrequency } from 'utils/utils'
import { HealthcheckContext, HealthcheckStatus } from 'providers/HealthcheckProvider'

const EegSimulatorPanel = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH}px;
  position: static;
  display: flex;
  flex-direction: column;
  gap: 4px;
`

const DatasetSelect = styled(Select)`
  margin-left: 6px;
  width: 150px;
`

const SwitchWrapper = styled.span`
  width: 59px;
  display: inline-flex;
  justify-content: flex-end;
`

const CompactRow = styled(ConfigRow)`
  margin-bottom: 2px;
  gap: 4px;
`

const SectionStartRow = styled(CompactRow)`
  margin-top: 6px;
`

export const EegSimulatorDisplay: React.FC = () => {
  const { eegSimulatorHealthcheck } = useContext(HealthcheckContext)
  const {
    datasetList,
    dataset,
    startTime,
    streamerState,
    toggleStreaming,
  } = useContext(EegSimulatorContext)
  const { experimentState } = useContext(PipelineContext)

  const eegSimulatorHealthcheckOk = eegSimulatorHealthcheck?.status.value === HealthcheckStatus.READY
  const isRunning = streamerState === StreamerStateValue.RUNNING
  const isLoading = streamerState === StreamerStateValue.LOADING
  const isExperimentOngoing = experimentState?.ongoing ?? false

  const setDataset = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const newDataset = event.target.value

    setDatasetRos(newDataset, () => {
      console.log('Dataset set to ' + newDataset)
    })
  }

  const setStartTime = (startTime: number) => {
    if (startTime < 0 || startTime > (selectedDataset?.duration || 0)) {
      console.error('Start time must be between 0 and ' + selectedDataset?.duration + ' seconds')
      return
    }
    setStartTimeRos(startTime, () => {
      console.log('Start time set to ' + startTime)
    })
  }

  const selectedDataset = datasetList.find(
    (d: typeof datasetList[number]) => d.json_filename === dataset
  )
  const ActionButton = isRunning ? StyledRedButton : StyledButton
  const actionLabel = isRunning ? 'Stop' : 'Start'
  const actionDisabled = !selectedDataset || isLoading
  const streamerStateLabel =
    streamerState === StreamerStateValue.RUNNING
      ? 'Running'
      : streamerState === StreamerStateValue.LOADING
      ? 'Loading'
      : streamerState === StreamerStateValue.ERROR
      ? 'Error'
      : 'Ready'

  return (
    <EegSimulatorPanel isGrayedOut={!eegSimulatorHealthcheckOk}>
      <SmallerTitle>Simulator</SmallerTitle>
      <ConfigRow style={{ justifyContent: 'flex-end', paddingRight: 12 }}>
        <ActionButton onClick={toggleStreaming} disabled={actionDisabled}>
          {actionLabel}
        </ActionButton>
      </ConfigRow>
      <ConfigRow style={{ justifyContent: 'space-between' }}>
        <ConfigLabel>Dataset:</ConfigLabel>
        <DatasetSelect onChange={setDataset} value={dataset} disabled={isExperimentOngoing}>
          {datasetList.map((dataset: typeof datasetList[number], index: number) => (
            <option key={index} value={dataset.json_filename}>
              {dataset.name}
            </option>
          ))}
        </DatasetSelect>
      </ConfigRow>
      <CompactRow>
        <ConfigLabel>Length:</ConfigLabel>
        <ConfigValue>
          {selectedDataset?.loop
            ? 'Continuous'
            : `${formatTime(selectedDataset?.duration)}${selectedDataset?.pulse_count ? `, ${selectedDataset.pulse_count} pulses` : ''}`}
        </ConfigValue>
      </CompactRow>
      <SectionStartRow>
        <ConfigLabel>Sampling rate:</ConfigLabel>
        <ConfigValue>{formatFrequency(selectedDataset?.sampling_frequency)}</ConfigValue>
      </SectionStartRow>
      <CompactRow>
        <ConfigLabel>Channels:</ConfigLabel>
      </CompactRow>
      <CompactRow>
        <ConfigLabel style={{ paddingLeft: 10 }}>EEG</ConfigLabel>
        <ConfigValue>{selectedDataset?.num_eeg_channels}</ConfigValue>
      </CompactRow>
      <CompactRow>
        <ConfigLabel style={{ paddingLeft: 10 }}>EMG</ConfigLabel>
        <ConfigValue>{selectedDataset?.num_emg_channels}</ConfigValue>
      </CompactRow>
      <CompactRow style={{ justifyContent: 'space-between' }}>
        <ConfigLabel>Start time (s)</ConfigLabel>
        <div style={{ marginRight: 20 }}>
          <ValidatedInput
            type='number'
            value={startTime}
            min={0}
            max={selectedDataset?.duration || 0}
            onChange={setStartTime}
            disabled={isExperimentOngoing}
          />
        </div>
      </CompactRow>
      <CompactRow>
        <ConfigLabel>Status:</ConfigLabel>
        <ConfigValue>{streamerStateLabel}</ConfigValue>
      </CompactRow>
    </EegSimulatorPanel>
  )
}
