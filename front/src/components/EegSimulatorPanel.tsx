import React, { useContext } from 'react'
import styled from 'styled-components'

import { ValidatedInput } from 'components/ValidatedInput'

import {
  StyledPanel,
  Select,
  ConfigRow,
  ConfigLabel,
  ConfigValue,
  CONFIG_PANEL_WIDTH,
  StyledButton,
  StyledRedButton,
} from 'styles/General'

import { EegSimulatorContext, StreamerStateValue } from 'providers/EegSimulatorProvider'
import { PipelineContext } from 'providers/PipelineProvider'
import { EegStreamContext } from 'providers/EegStreamProvider'
import { useParameters } from 'providers/ParameterProvider'
import { formatTime, formatFrequency } from 'utils/utils'
import { HealthcheckContext, HealthcheckStatus } from 'providers/HealthcheckProvider'

const SimulatorPanel = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH - 30}px;
  position: static;
  display: flex;
  flex-direction: column;
  gap: 4px;
`

const DatasetSelect = styled(Select)`
  margin-left: 6px;
  width: 150px;
`

const CompactRow = styled(ConfigRow)`
  margin-bottom: 2px;
  gap: 4px;
`

const SectionStartRow = styled(CompactRow)`
  margin-top: 6px;
`

export const EegSimulatorPanel: React.FC<{ isGrayedOut: boolean }> = ({ isGrayedOut }) => {
  const { eegSimulatorHealthcheck } = useContext(HealthcheckContext)
  const {
    datasetList,
    dataset,
    startTime,
    streamerState,
  } = useContext(EegSimulatorContext)
  const { experimentState } = useContext(PipelineContext)
  const { eegInfo } = useContext(EegStreamContext)
  const { setSimulatorDataset, setSimulatorStartTime } = useParameters()

  const eegSimulatorHealthcheckOk = eegSimulatorHealthcheck?.status.value === HealthcheckStatus.READY
  const isRunning = streamerState === StreamerStateValue.RUNNING
  const isLoading = streamerState === StreamerStateValue.LOADING
  const isExperimentOngoing = experimentState?.ongoing ?? false
  const isEegStreaming = eegInfo?.is_streaming || false

  const setDataset = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const newDataset = event.target.value

    setSimulatorDataset(newDataset, () => {
      console.log('Dataset set to ' + newDataset)
    })
  }

  const setStartTime = (startTime: number) => {
    if (startTime < 0 || startTime > (selectedDataset?.duration || 0)) {
      console.error('Start time must be between 0 and ' + selectedDataset?.duration + ' seconds')
      return
    }
    setSimulatorStartTime(startTime, () => {
      console.log('Start time set to ' + startTime)
    })
  }

  /*
  const selectedDataset = datasetList.find(
    (d: typeof datasetList[number]) => d.json_filename === dataset
  )
  */
  // Just a dummy for now
  const selectedDataset = {
    name: 'Dummy Dataset',
    json_filename: 'dummy_dataset.json',
    duration: 100,
    sampling_frequency: 100,
    num_eeg_channels: 1,
    num_emg_channels: 0,
    loop: false,
    pulse_count: 0,
  }
    

  const streamerStateLabel =
    streamerState === StreamerStateValue.RUNNING
      ? 'Running'
      : streamerState === StreamerStateValue.LOADING
      ? 'Loading'
      : streamerState === StreamerStateValue.ERROR
      ? 'Error'
      : 'Ready'

  return (
    <SimulatorPanel isGrayedOut={isGrayedOut}>
      <ConfigRow style={{ justifyContent: 'space-between' }}>
        <ConfigLabel>Dataset:</ConfigLabel>
        <DatasetSelect onChange={setDataset} value={dataset} disabled={isExperimentOngoing || isEegStreaming}>
          {datasetList.map((datasetFilename: typeof datasetList[number], index: number) => (
            <option key={index} value={datasetFilename}>
              {datasetFilename}
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
            disabled={isExperimentOngoing || isEegStreaming}
          />
        </div>
      </CompactRow>
      <CompactRow>
        <ConfigLabel>Status:</ConfigLabel>
        <ConfigValue>{streamerStateLabel}</ConfigValue>
      </CompactRow>
    </SimulatorPanel>
  )
}