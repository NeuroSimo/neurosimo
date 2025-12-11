import React, { useContext } from 'react'
import styled from 'styled-components'

import {
  StyledPanel,
  SmallerTitle,
  ConfigRow,
  ConfigLabel,
  ConfigValue,
  CONFIG_PANEL_WIDTH,
} from 'styles/General'

import { EegStreamContext } from 'providers/EegStreamProvider'
import { HealthcheckContext, HealthcheckStatus } from 'providers/HealthcheckProvider'

import { formatFrequency } from 'utils/utils'

const EegDevicePanel = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH}px;
  position: static;
  display: flex;
  flex-direction: column;
  gap: 4px;
`

const CompactRow = styled(ConfigRow)`
  margin-bottom: 2px;
  gap: 4px;
`

export const EegDeviceDisplay: React.FC = () => {
  const { eegInfo } = useContext(EegStreamContext)
  const { eegHealthcheck } = useContext(HealthcheckContext)

  const eegHealthcheckOk = eegHealthcheck?.status.value === HealthcheckStatus.READY
  const numOfEegChannels = eegInfo?.num_eeg_channels || 0
  const numOfEmgChannels = eegInfo?.num_emg_channels || 0

  return (
    <EegDevicePanel isGrayedOut={!eegHealthcheckOk}>
      <SmallerTitle>EEG Device</SmallerTitle>
      <ConfigRow>
        <ConfigLabel>Sampling rate:</ConfigLabel>
        <ConfigValue>{formatFrequency(eegInfo?.sampling_frequency)}</ConfigValue>
      </ConfigRow>
      <CompactRow>
        <ConfigLabel>Channels:</ConfigLabel>
      </CompactRow>
      <CompactRow>
        <ConfigLabel style={{ paddingLeft: 10 }}>EEG</ConfigLabel>
        <ConfigValue>{numOfEegChannels > 0 ? numOfEegChannels : '\u2013'}</ConfigValue>
      </CompactRow>
      <CompactRow>
        <ConfigLabel style={{ paddingLeft: 10 }}>EMG</ConfigLabel>
        <ConfigValue>{numOfEmgChannels > 0 ? numOfEmgChannels : '\u2013'}</ConfigValue>
      </CompactRow>
    </EegDevicePanel>
  )
}