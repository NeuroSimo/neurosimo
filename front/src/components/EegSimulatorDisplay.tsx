import React, { useContext, useEffect, useState } from 'react'
import styled from 'styled-components'

import { ToggleSwitch } from 'components/ToggleSwitch'
import { ValidatedInput } from 'components/ValidatedInput'

import {
  StyledPanel,
  Select,
  GrayedOutPanel,
  SmallerTitle,
  ConfigRow,
  ConfigLabel,
  ConfigValue,
  CONFIG_PANEL_WIDTH,
} from 'styles/General'

import { EegSimulatorContext } from 'providers/EegSimulatorProvider'
import { setDatasetRos, setPlaybackRos, setLoopRos, setStartTimeRos, setRecordDataRos } from 'ros/eeg_simulator'
import { formatTime, formatFrequency } from 'utils/utils'
import { HealthcheckContext, HealthcheckStatus } from 'providers/HealthcheckProvider'

const EegSimulatorPanel = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH}px;
  position: static;
  display: flex;
  flex-direction: column;
  gap: 8px;
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
    <EegSimulatorPanel isGrayedOut={!eegSimulatorHealthcheckOk}>
      <SmallerTitle>Simulator</SmallerTitle>
      <ConfigRow style={{ justifyContent: 'space-between' }}>
        <ConfigLabel>Dataset:</ConfigLabel>
        <DatasetSelect onChange={setDataset} value={dataset}>
          {datasetList.map((dataset, index) => (
            <option key={index} value={dataset.json_filename}>
              {dataset.name}
            </option>
          ))}
        </DatasetSelect>
      </ConfigRow>
      <br />
      <CompactRow>
        <ConfigLabel>Duration:</ConfigLabel>
        <ConfigValue>{formatTime(selectedDataset?.duration)}</ConfigValue>
      </CompactRow>
      <CompactRow>
        <ConfigLabel>Sampling rate:</ConfigLabel>
        <ConfigValue>{formatFrequency(selectedDataset?.sampling_frequency)}</ConfigValue>
      </CompactRow>
      <CompactRow>
        <ConfigLabel>Channels:</ConfigLabel>
      </CompactRow>
      <CompactRow>
        <ConfigLabel style={{ paddingLeft: 10 }}>EEG</ConfigLabel>
        <ConfigValue>{selectedDataset?.num_of_eeg_channels}</ConfigValue>
      </CompactRow>
      <CompactRow>
        <ConfigLabel style={{ paddingLeft: 10 }}>EMG</ConfigLabel>
        <ConfigValue>{selectedDataset?.num_of_emg_channels}</ConfigValue>
      </CompactRow>
      <br />
      <CompactRow style={{ justifyContent: 'space-between' }}>
        <ConfigLabel>Playback:</ConfigLabel>
        <SwitchWrapper>
          <ToggleSwitch type='flat' checked={playback} onChange={setPlayback} disabled={false} />
        </SwitchWrapper>
      </CompactRow>
      <GrayedOutPanel isGrayedOut={!playback}>
        <CompactRow style={{ justifyContent: 'space-between' }}>
          <ConfigLabel style={{ paddingLeft: 10 }}>Loop</ConfigLabel>
          <SwitchWrapper>
            <ToggleSwitch type='flat' checked={loop} onChange={setLoop} disabled={!playback} />
          </SwitchWrapper>
        </CompactRow>
        <CompactRow style={{ justifyContent: 'space-between' }}>
          <ConfigLabel style={{ paddingLeft: 10 }}>Record</ConfigLabel>
          <SwitchWrapper>
            <ToggleSwitch type='flat' checked={recordData} onChange={setRecordData} disabled={!playback} />
          </SwitchWrapper>
        </CompactRow>
        <CompactRow style={{ justifyContent: 'space-between' }}>
          <ConfigLabel style={{ paddingLeft: 10 }}>Start time (s)</ConfigLabel>
          <div style={{ marginRight: 20 }}>
            <ValidatedInput
              type='number'
              value={startTime}
              min={0}
              max={selectedDataset?.duration || 0}
              onChange={setStartTime}
              disabled={!playback}
            />
          </div>
        </CompactRow>
      </GrayedOutPanel>
      <br />
      <br />
      <ConfigRow>
        <ConfigLabel>Status:</ConfigLabel>
        <ConfigValue>{eegSimulatorHealthcheck?.status_message}</ConfigValue>
      </ConfigRow>
    </EegSimulatorPanel>
  )
}
