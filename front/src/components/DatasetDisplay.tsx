import React, { useContext, useEffect, useState } from 'react'
import styled from 'styled-components'

import { ToggleSwitch } from 'components/Experiment/ToggleSwitch'

import {
  IndentedStateTitle,
  StyledPanel,
  StateRow,
  StateTitle,
  StateValue,
  Select,
  GrayedOutPanel,
} from 'styles/General'

import { DatasetContext } from 'providers/DatasetProvider'
import { setDatasetRos, setPlaybackRos, setLoopRos, setRecordDataRos } from 'ros/ros'
import { formatTime, formatFrequency } from 'utils/utils'
import { HealthcheckContext, HealthcheckStatus } from 'providers/HealthcheckProvider'

const DatasetPanelTitle = styled.div`
  width: 340px;
  position: fixed;
  top: 896px;
  right: 380px;
  z-index: 1001;
  text-align: left;
  font-size: 20px;
  font-weight: bold;
`

const DatasetPanel = styled(StyledPanel)`
  width: 300px;
  height: 356px;
  position: fixed;
  top: 928px;
  right: 380px;
  z-index: 1000;
`

const DatasetSelect = styled(Select)`
  margin-left: 10px;
  width: 210px;
`

const SwitchWrapper = styled.span`
  width: 95px;
`

export const DatasetDisplay: React.FC = () => {
  const { eegSimulatorHealthcheck } = useContext(HealthcheckContext)
  const { datasetList, dataset, playback, loop, recordData } = useContext(DatasetContext)

  const eegSimulatorHealthcheckOk = eegSimulatorHealthcheck?.status.value === HealthcheckStatus.READY

  const handleDatasetChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const newDataset = event.target.value

    setDatasetRos(newDataset, () => {
      console.log('Dataset set to ' + newDataset)
    })
  }

  const handlePlaybackChange = (playback: boolean) => {
    setPlaybackRos(playback, () => {
      console.log('Playback set to ' + playback)
    })
  }

  const handleLoopChange = (loop: boolean) => {
    setLoopRos(loop, () => {
      console.log('Loop set to ' + loop)
    })
  }

  const handleRecordDataChange = (recordData: boolean) => {
    setRecordDataRos(recordData, () => {
      console.log('Record data set to ' + recordData)
    })
  }

  const selectedDataset = datasetList.find((d) => d.json_filename === dataset)

  return (
    <>
      <DatasetPanelTitle>Simulator</DatasetPanelTitle>
      <DatasetPanel isGrayedOut={!eegSimulatorHealthcheckOk}>
        <StateRow>
          <StateTitle>Dataset:</StateTitle>
          <StateValue>
            <DatasetSelect onChange={handleDatasetChange} value={dataset}>
              {datasetList.map((dataset, index) => (
                <option key={index} value={dataset.json_filename}>
                  {dataset.name}
                </option>
              ))}
            </DatasetSelect>
          </StateValue>
        </StateRow>
        <br />
        <StateRow>
          <StateTitle>Duration:</StateTitle>
          <StateValue>{formatTime(selectedDataset?.duration)}</StateValue>
        </StateRow>
        <StateRow>
          <StateTitle>Sampling rate:</StateTitle>
          <StateValue>{formatFrequency(selectedDataset?.sampling_frequency)}</StateValue>
        </StateRow>
        <StateRow>
          <StateTitle>Channels:</StateTitle>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>EEG</IndentedStateTitle>
          <StateValue>{selectedDataset?.num_of_eeg_channels}</StateValue>
        </StateRow>
        <StateRow>
          <IndentedStateTitle>EMG</IndentedStateTitle>
          <StateValue>{selectedDataset?.num_of_emg_channels}</StateValue>
        </StateRow>
        <br />
        <StateRow>
          <StateTitle>Playback:</StateTitle>
          <SwitchWrapper>
            <ToggleSwitch type='flat' checked={playback} onChange={handlePlaybackChange} disabled={false} />
          </SwitchWrapper>
        </StateRow>
        <GrayedOutPanel isGrayedOut={!playback}>
          <StateRow>
            <IndentedStateTitle>Loop</IndentedStateTitle>
            <SwitchWrapper>
              <ToggleSwitch type='flat' checked={loop} onChange={handleLoopChange} disabled={!playback} />
            </SwitchWrapper>
          </StateRow>
          <StateRow>
            <IndentedStateTitle>Record</IndentedStateTitle>
            <SwitchWrapper>
              <ToggleSwitch type='flat' checked={recordData} onChange={handleRecordDataChange} disabled={!playback} />
            </SwitchWrapper>
          </StateRow>
        </GrayedOutPanel>
        <br />
        <br />
        <StateRow>
          <StateTitle>Status:</StateTitle>
          <StateValue>{eegSimulatorHealthcheck?.status_message}</StateValue>
        </StateRow>
      </DatasetPanel>
    </>
  )
}
