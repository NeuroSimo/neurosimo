import React, { useContext, useState, useEffect } from 'react'
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
import { getDatasetInfoRos, DatasetInfo } from 'ros/eeg_simulator'

const SimulatorPanel = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH - 30}px;
  position: static;
  display: flex;
  flex-direction: column;
  gap: 4px;
`

const DatasetSelect = styled(Select)`
  margin-left: 6px;
  width: 170px;
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

  const [selectedDatasetInfo, setSelectedDatasetInfo] = useState<DatasetInfo | null>(null)

  const eegSimulatorHealthcheckOk = eegSimulatorHealthcheck?.status.value === HealthcheckStatus.READY
  const isRunning = streamerState === StreamerStateValue.RUNNING
  const isLoading = streamerState === StreamerStateValue.LOADING
  const isExperimentOngoing = experimentState?.ongoing ?? false
  const isEegStreaming = eegInfo?.is_streaming || false

  // Fetch dataset info when dataset changes
  useEffect(() => {
    if (!dataset || dataset.trim() === '') {
      setSelectedDatasetInfo(null)
      return
    }
    getDatasetInfoRos(dataset, (datasetInfo) => {
      if (!datasetInfo) {
        console.error('Failed to get dataset info for:', dataset)
        setSelectedDatasetInfo(null)
        return
      }
      setSelectedDatasetInfo(datasetInfo)
    })
  }, [dataset])

  const setDataset = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const newDataset = event.target.value

    setSimulatorDataset(newDataset, () => {
      console.log('Dataset set to ' + newDataset)
    })
  }

  const setStartTime = (startTime: number) => {
    if (startTime < 0 || startTime > (selectedDatasetInfo?.duration || 0)) {
      console.error('Start time must be between 0 and ' + selectedDatasetInfo?.duration + ' seconds')
      return
    }
    setSimulatorStartTime(startTime, () => {
      console.log('Start time set to ' + startTime)
    })
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
          {selectedDatasetInfo?.loop
            ? 'Continuous'
            : `${formatTime(selectedDatasetInfo?.duration)}${selectedDatasetInfo?.pulse_count ? `, ${selectedDatasetInfo.pulse_count} pulses` : ''}`}
        </ConfigValue>
      </CompactRow>
      <SectionStartRow>
        <ConfigLabel>Sampling rate:</ConfigLabel>
        <ConfigValue>{formatFrequency(selectedDatasetInfo?.sampling_frequency)}</ConfigValue>
      </SectionStartRow>
      <CompactRow>
        <ConfigLabel>Channels:</ConfigLabel>
      </CompactRow>
      <CompactRow>
        <ConfigLabel style={{ paddingLeft: 10 }}>EEG</ConfigLabel>
        <ConfigValue>{selectedDatasetInfo?.num_eeg_channels}</ConfigValue>
      </CompactRow>
      <CompactRow>
        <ConfigLabel style={{ paddingLeft: 10 }}>EMG</ConfigLabel>
        <ConfigValue>{selectedDatasetInfo?.num_emg_channels}</ConfigValue>
      </CompactRow>
      <CompactRow style={{ justifyContent: 'space-between' }}>
        <ConfigLabel>Start time (s)</ConfigLabel>
        <div style={{ marginRight: 20 }}>
          <ValidatedInput
            type='number'
            value={startTime}
            min={0}
            max={selectedDatasetInfo?.duration || 0}
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