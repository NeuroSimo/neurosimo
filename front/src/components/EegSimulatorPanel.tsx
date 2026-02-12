import React, { useContext, useState, useEffect } from 'react'
import styled from 'styled-components'

import { ValidatedInput } from 'components/ValidatedInput'
import { FolderTerminalButtons } from 'components/FolderTerminalButtons'

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

import { EegSimulatorContext, DataSourceStateValue } from 'providers/EegSimulatorProvider'
import { ExperimentContext } from 'providers/ExperimentProvider'
import { EegStreamContext } from 'providers/EegStreamProvider'
import { useSessionConfig } from 'providers/SessionConfigProvider'
import { useSession, SessionStateValue } from 'providers/SessionProvider'
import { formatTime, formatFrequency } from 'utils/utils'
import { HealthcheckContext } from 'providers/HealthProvider'
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


export const EegSimulatorPanel: React.FC<{ isGrayedOut: boolean }> = ({ isGrayedOut }) => {
  const { eegSimulatorStatus } = useContext(HealthcheckContext)
  const {
    datasetList,
    dataset,
    startTime,
    dataSourceState,
  } = useContext(EegSimulatorContext)
  const { experimentState } = useContext(ExperimentContext)
  const { eegDeviceInfo } = useContext(EegStreamContext)
  const { setSimulatorDataset, setSimulatorStartTime } = useSessionConfig()
  const { sessionState } = useSession()

  const [selectedDatasetInfo, setSelectedDatasetInfo] = useState<DatasetInfo | null>(null)

  const isSessionRunning = sessionState.state === SessionStateValue.RUNNING
  const isEegStreaming = eegDeviceInfo?.is_streaming || false

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

  // Handle arrow key navigation for dataset selection
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      // Only handle arrow keys when not typing in inputs and not disabled
      if (isSessionRunning || isEegStreaming) return

      // Skip if user is typing in an input field
      const target = event.target as HTMLElement
      if (target.tagName === 'INPUT' || target.tagName === 'TEXTAREA') return

      const currentIndex = datasetList.indexOf(dataset)
      if (currentIndex === -1) return

      if (event.key === 'ArrowUp' && currentIndex > 0) {
        event.preventDefault()
        // Blur any currently focused element to prevent focus outline
        if (document.activeElement instanceof HTMLElement) {
          document.activeElement.blur()
        }
        const newIndex = currentIndex - 1
        setSimulatorDataset(datasetList[newIndex], () => {
          console.log('Dataset changed to ' + datasetList[newIndex] + ' via arrow key')
        })
      } else if (event.key === 'ArrowDown' && currentIndex < datasetList.length - 1) {
        event.preventDefault()
        // Blur any currently focused element to prevent focus outline
        if (document.activeElement instanceof HTMLElement) {
          document.activeElement.blur()
        }
        const newIndex = currentIndex + 1
        setSimulatorDataset(datasetList[newIndex], () => {
          console.log('Dataset changed to ' + datasetList[newIndex] + ' via arrow key')
        })
      }
    }

    document.addEventListener('keydown', handleKeyDown)
    return () => document.removeEventListener('keydown', handleKeyDown)
  }, [dataset, datasetList, isSessionRunning, isEegStreaming, setSimulatorDataset])

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

  const dataSourceStateLabel =
    dataSourceState === DataSourceStateValue.RUNNING
      ? 'Running'
      : dataSourceState === DataSourceStateValue.LOADING
      ? 'Loading'
      : dataSourceState === DataSourceStateValue.ERROR
      ? 'Error'
      : 'Ready'

  return (
    <SimulatorPanel isGrayedOut={isGrayedOut}>
      <ConfigRow style={{ justifyContent: 'space-between' }}>
        <ConfigLabel>Dataset</ConfigLabel>
        <DatasetSelect onChange={setDataset} value={dataset} disabled={isSessionRunning || isEegStreaming}>
          {datasetList.map((datasetFilename: typeof datasetList[number], index: number) => (
            <option key={index} value={datasetFilename}>
              {datasetFilename}
            </option>
          ))}
        </DatasetSelect>
      </ConfigRow>
      <CompactRow>
        <ConfigLabel>Duration</ConfigLabel>
        <ConfigValue>
          {selectedDatasetInfo?.loop
            ? 'Continuous'
            : `${formatTime(selectedDatasetInfo?.duration)}${selectedDatasetInfo?.pulse_count ? `, ${selectedDatasetInfo.pulse_count} pulses` : ''}`}
        </ConfigValue>
      </CompactRow>

      <div style={{ height: '8px' }} />

      <CompactRow>
        <ConfigLabel>Sampling rate</ConfigLabel>
        <ConfigValue>{formatFrequency(selectedDatasetInfo?.sampling_frequency)}</ConfigValue>
      </CompactRow>
      <CompactRow>
        <ConfigLabel>Channels</ConfigLabel>
      </CompactRow>
      <CompactRow>
        <ConfigLabel style={{ paddingLeft: 10 }}>EEG</ConfigLabel>
        <ConfigValue>{selectedDatasetInfo?.num_eeg_channels}</ConfigValue>
      </CompactRow>
      <CompactRow>
        <ConfigLabel style={{ paddingLeft: 10 }}>EMG</ConfigLabel>
        <ConfigValue>{selectedDatasetInfo?.num_emg_channels}</ConfigValue>
      </CompactRow>

      <div style={{ height: '8px' }} />

      <CompactRow style={{ justifyContent: 'space-between' }}>
        <ConfigLabel>Start time (s)</ConfigLabel>
        <div style={{ marginRight: 20 }}>
          <ValidatedInput
            type='number'
            value={startTime}
            min={0}
            max={selectedDatasetInfo?.duration || 0}
            onChange={setStartTime}
            disabled={isSessionRunning || isEegStreaming}
            width="60px"
          />
        </div>
      </CompactRow>
      <CompactRow>
        <ConfigLabel>Status</ConfigLabel>
        <ConfigValue>{dataSourceStateLabel}</ConfigValue>
      </CompactRow>
      <div style={{ height: '8px' }} />
      <CompactRow style={{ justifyContent: 'flex-end', paddingRight: '10px' }}>
        <FolderTerminalButtons folderName="eeg_simulator" />
      </CompactRow>
    </SimulatorPanel>
  )
}