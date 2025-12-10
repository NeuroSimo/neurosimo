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
import { setDatasetRos, setEnabledRos, setStartTimeRos, setRecordDataRos } from 'ros/eeg_simulator'
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
  const { datasetList, dataset, enabled, recordData, startTime } = useContext(EegSimulatorContext)

  const eegSimulatorHealthcheckOk = eegSimulatorHealthcheck?.status.value === HealthcheckStatus.READY

  const setDataset = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const newDataset = event.target.value

    setDatasetRos(newDataset, () => {
      console.log('Dataset set to ' + newDataset)
    })
  }

  const setEnabled = (enabled: boolean) => {
    setEnabledRos(enabled, () => {
      console.log('Enabled set to ' + enabled)
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
      <SectionStartRow style={{ justifyContent: 'space-between' }}>
        <ConfigLabel>Enabled:</ConfigLabel>
        <SwitchWrapper>
          <ToggleSwitch type='flat' checked={enabled} onChange={setEnabled} disabled={false} />
        </SwitchWrapper>
      </SectionStartRow>
      <GrayedOutPanel isGrayedOut={!enabled}>
        <CompactRow style={{ justifyContent: 'space-between' }}>
          <ConfigLabel style={{ paddingLeft: 10 }}>Record</ConfigLabel>
          <SwitchWrapper>
            <ToggleSwitch type='flat' checked={recordData} onChange={setRecordData} disabled={!enabled} />
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
              disabled={!enabled}
            />
          </div>
        </CompactRow>
      </GrayedOutPanel>
      <SectionStartRow>
        <ConfigLabel>Status:</ConfigLabel>
        <ConfigValue>{eegSimulatorHealthcheck?.status_message}</ConfigValue>
      </SectionStartRow>
    </EegSimulatorPanel>
  )
}
