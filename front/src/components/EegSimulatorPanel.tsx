import React, { useContext, useState, useEffect } from 'react'
import styled from 'styled-components'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'
import { faFolderOpen } from '@fortawesome/free-solid-svg-icons'

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

import { EegSimulatorContext, DataSourceStateValue } from 'providers/EegSimulatorProvider'
import { PipelineContext } from 'providers/PipelineProvider'
import { EegStreamContext } from 'providers/EegStreamProvider'
import { ProjectContext } from 'providers/ProjectProvider'
import { useParameters } from 'providers/ParameterProvider'
import { useSession, SessionStage } from 'providers/SessionProvider'
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

const OpenFolderButton = styled.button<{ disabled: boolean }>`
  background-color: ${props => props.disabled ? '#cccccc' : '#28a745'};
  color: ${props => props.disabled ? '#666666' : 'white'};
  border: none;
  border-radius: 4px;
  padding: 6px 8px;
  cursor: ${props => props.disabled ? 'not-allowed' : 'pointer'};
  font-size: 14px;
  display: flex;
  align-items: center;
  justify-content: center;
  margin-left: auto;
  margin-right: 10px;

  &:hover {
    background-color: ${props => props.disabled ? '#cccccc' : '#218838'};
  }
`


export const EegSimulatorPanel: React.FC<{ isGrayedOut: boolean }> = ({ isGrayedOut }) => {
  const { eegSimulatorStatus } = useContext(HealthcheckContext)
  const {
    datasetList,
    dataset,
    startTime,
    dataSourceState,
  } = useContext(EegSimulatorContext)
  const { experimentState } = useContext(PipelineContext)
  const { eegDeviceInfo } = useContext(EegStreamContext)
  const { activeProject } = useContext(ProjectContext)
  const { setSimulatorDataset, setSimulatorStartTime } = useParameters()
  const { sessionState } = useSession()

  const [selectedDatasetInfo, setSelectedDatasetInfo] = useState<DatasetInfo | null>(null)

  const isSessionRunning = sessionState.stage !== SessionStage.STOPPED
  const isEegStreaming = eegDeviceInfo?.is_streaming || false
  const isElectron = !!(window as any).electronAPI

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

  const handleOpenFolder = async () => {
    if (!activeProject) return
    
    const error = await (window as any).electronAPI?.openProjectFolder(activeProject, 'eeg_simulator')
    if (error) console.error('Failed to open folder:', error)
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
        <ConfigLabel>Dataset:</ConfigLabel>
        <DatasetSelect onChange={setDataset} value={dataset} disabled={isSessionRunning || isEegStreaming}>
          {datasetList.map((datasetFilename: typeof datasetList[number], index: number) => (
            <option key={index} value={datasetFilename}>
              {datasetFilename}
            </option>
          ))}
        </DatasetSelect>
      </ConfigRow>
      <CompactRow>
        <ConfigLabel>Duration:</ConfigLabel>
        <ConfigValue>
          {selectedDatasetInfo?.loop
            ? 'Continuous'
            : `${formatTime(selectedDatasetInfo?.duration)}${selectedDatasetInfo?.pulse_count ? `, ${selectedDatasetInfo.pulse_count} pulses` : ''}`}
        </ConfigValue>
      </CompactRow>

      <div style={{ height: '8px' }} />

      <CompactRow>
        <ConfigLabel>Sampling rate:</ConfigLabel>
        <ConfigValue>{formatFrequency(selectedDatasetInfo?.sampling_frequency)}</ConfigValue>
      </CompactRow>
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
          />
        </div>
      </CompactRow>
      <CompactRow>
        <ConfigLabel>Status:</ConfigLabel>
        <ConfigValue>{dataSourceStateLabel}</ConfigValue>
      </CompactRow>
      <div style={{ height: '8px' }} />
      <CompactRow>
        <OpenFolderButton
          disabled={!activeProject || !isElectron}
          onClick={handleOpenFolder}
          title={isElectron ? "Open EEG simulator folder" : "Only available in Electron"}
        >
          <FontAwesomeIcon icon={faFolderOpen} />
        </OpenFolderButton>
      </CompactRow>
    </SimulatorPanel>
  )
}