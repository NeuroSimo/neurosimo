import React, { useContext } from 'react'
import styled from 'styled-components'

import {
  StyledPanel,
  ConfigRow,
  ConfigLabel,
  ConfigValue,
  CONFIG_PANEL_WIDTH,
  StyledButton,
  StyledRedButton,
} from 'styles/General'


import { EegStreamContext } from 'providers/EegStreamProvider'
import { EegBridgeContext, EegBridgeStateValue } from 'providers/EegBridgeProvider'
import { formatFrequency } from 'utils/utils'

const EegDeviceContainer = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH - 30}px;
  position: static;
  display: flex;
  flex-direction: column;
  gap: 4px;
`

const CompactRow = styled(ConfigRow)`
  margin-bottom: 2px;
  gap: 4px;
`

export const EegDevicePanel: React.FC = () => {
  const { eegDeviceInfo } = useContext(EegStreamContext)
  const { bridgeState } = useContext(EegBridgeContext)

  const samplingFrequency = eegDeviceInfo?.sampling_frequency ? formatFrequency(eegDeviceInfo.sampling_frequency) : '\u2013'
  const numEegChannels = eegDeviceInfo?.num_eeg_channels ? eegDeviceInfo.num_eeg_channels : '\u2013'
  const numEmgChannels = eegDeviceInfo?.num_emg_channels ? eegDeviceInfo.num_emg_channels : '\u2013'

  const isRunning = bridgeState === EegBridgeStateValue.RUNNING
  const isLoading = bridgeState === EegBridgeStateValue.LOADING

  const dataSourceStateLabel =
    bridgeState === EegBridgeStateValue.RUNNING
      ? 'Running'
      : bridgeState === EegBridgeStateValue.LOADING
      ? 'Loading'
      : bridgeState === EegBridgeStateValue.ERROR
      ? 'Error'
      : 'Ready'

  return (
    <EegDeviceContainer>
      <ConfigRow>
        <ConfigLabel>Sampling rate:</ConfigLabel>
        <ConfigValue>{samplingFrequency}</ConfigValue>
      </ConfigRow>
      <CompactRow>
        <ConfigLabel>Channels:</ConfigLabel>
      </CompactRow>
      <CompactRow>
        <ConfigLabel style={{ paddingLeft: 10 }}>EEG</ConfigLabel>
        <ConfigValue>{numEegChannels > 0 ? numEegChannels : '\u2013'}</ConfigValue>
      </CompactRow>
      <CompactRow>
        <ConfigLabel style={{ paddingLeft: 10 }}>EMG</ConfigLabel>
        <ConfigValue>{numEmgChannels > 0 ? numEmgChannels : '\u2013'}</ConfigValue>
      </CompactRow>
      <CompactRow>
        <ConfigLabel>Status:</ConfigLabel>
        <ConfigValue>{dataSourceStateLabel}</ConfigValue>
      </CompactRow>
    </EegDeviceContainer>
  )
}