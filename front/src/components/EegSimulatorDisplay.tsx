import React, { useContext, useEffect, useState } from 'react'
import styled from 'styled-components'

import { ToggleSwitch } from 'components/ToggleSwitch'
import { ValidatedInput } from 'components/ValidatedInput'

import {
  IndentedStateTitle,
  StyledPanel,
  StateRow,
  StateTitle,
  StateValue,
  Select,
  GrayedOutPanel,
} from 'styles/General'

import { EegSimulatorContext } from 'providers/EegSimulatorProvider'
import { setDatasetRos, setPlaybackRos, setLoopRos, setStartTimeRos, setRecordDataRos } from 'ros/eeg_simulator'
import { formatTime, formatFrequency } from 'utils/utils'
import { HealthcheckContext, HealthcheckStatus } from 'providers/HealthcheckProvider'

const EegSimulatorPanelTitle = styled.div`
  width: 340px;
  position: fixed;
  top: 686px;
  right: 780px;
  z-index: 1001;
  text-align: left;
  font-size: 20px;
  font-weight: bold;
`

const EegSimulatorPanel = styled(StyledPanel)`
  width: 300px;
  height: 406px;
  position: fixed;
  top: 718px;
  right: 780px;
  z-index: 1000;
`

const DatasetSelect = styled(Select)`
  margin-left: 10px;
  width: 210px;
`

const SwitchWrapper = styled.span`
  width: 95px;
`

export const EegSimulatorDisplay: React.FC = () => {
  const { eegSimulatorHealthcheck } = useContext(HealthcheckContext)
  const { datasetList, dataset, playback, loop, recordData, startTime } = useContext(EegSimulatorContext)

  const eegSimulatorHealthcheckOk = eegSimulatorHealthcheck?.status.value === HealthcheckStatus.READY

  const setDataset = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const newDataset = event.target.value

    setDatasetRos(newDataset, () => {
      console.log('Dataset set to ' + newDataset)
    })
  }

  const setPlayback = (playback: boolean) => {
    setPlaybackRos(playback, () => {
      console.log('Playback set to ' + playback)
    })
  }

  const setLoop = (loop: boolean) => {
    setLoopRos(loop, () => {
      console.log('Loop set to ' + loop)
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

  const setRecordData = (recordData: boolean) => {
    setRecordDataRos(recordData, () => {
      console.log('Record data set to ' + recordData)
    })
  }

  const selectedDataset = datasetList.find((d) => d.json_filename === dataset)

  return (
    <>
      <EegSimulatorPanelTitle>Simulator</EegSimulatorPanelTitle>
      <EegSimulatorPanel isGrayedOut={!eegSimulatorHealthcheckOk}>
        <StateRow>
          <StateTitle>Dataset:</StateTitle>
          <StateValue>
            <DatasetSelect onChange={setDataset} value={dataset}>
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
            <ToggleSwitch type='flat' checked={playback} onChange={setPlayback} disabled={false} />
          </SwitchWrapper>
        </StateRow>
        <GrayedOutPanel isGrayedOut={!playback}>
          <StateRow>
            <IndentedStateTitle>Loop</IndentedStateTitle>
            <SwitchWrapper>
              <ToggleSwitch type='flat' checked={loop} onChange={setLoop} disabled={!playback} />
            </SwitchWrapper>
          </StateRow>
          <StateRow>
            <IndentedStateTitle>Record</IndentedStateTitle>
            <SwitchWrapper>
              <ToggleSwitch type='flat' checked={recordData} onChange={setRecordData} disabled={!playback} />
            </SwitchWrapper>
          </StateRow>
          <StateRow>
            <IndentedStateTitle>Start time (s)</IndentedStateTitle>
            <ValidatedInput
              type='number'
              value={startTime}
              min={0}
              max={selectedDataset?.duration || 0}
              onChange={setStartTime}
              disabled={!playback}
            />
          </StateRow>
        </GrayedOutPanel>
        <br />
        <br />
        <StateRow>
          <StateTitle>Status:</StateTitle>
          <StateValue>{eegSimulatorHealthcheck?.status_message}</StateValue>
        </StateRow>
      </EegSimulatorPanel>
    </>
  )
}
