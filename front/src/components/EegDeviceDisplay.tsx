import React, { useContext } from 'react'
import styled from 'styled-components'

import {
  StyledPanel,
  SmallerTitle,
  ConfigRow,
  ConfigLabel,
  ConfigValue,
  CONFIG_PANEL_WIDTH,
  StyledButton,
  StyledRedButton,
} from 'styles/General'

import { EegStreamContext } from 'providers/EegStreamProvider'
import { EegBridgeContext, EegBridgeStateValue } from 'providers/EegBridgeProvider'
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
  const { bridgeState, toggleStreaming } = useContext(EegBridgeContext)

  const eegHealthcheckOk = eegHealthcheck?.status.value === HealthcheckStatus.READY
  const numOfEegChannels = eegInfo?.num_eeg_channels || 0
  const numOfEmgChannels = eegInfo?.num_emg_channels || 0

  const isRunning = bridgeState === EegBridgeStateValue.RUNNING
  const isLoading = bridgeState === EegBridgeStateValue.LOADING

  const ActionButton = isRunning ? StyledRedButton : StyledButton
  const actionLabel = isRunning ? 'Stop' : 'Start'
  const actionDisabled = !eegHealthcheckOk || isLoading

  const streamerStateLabel =
    bridgeState === EegBridgeStateValue.RUNNING
      ? 'Running'
      : bridgeState === EegBridgeStateValue.LOADING
      ? 'Loading'
      : bridgeState === EegBridgeStateValue.ERROR
      ? 'Error'
      : 'Ready'

  return (
    <EegDevicePanel isGrayedOut={!eegHealthcheckOk}>
      <SmallerTitle>EEG Device</SmallerTitle>
      <ConfigRow style={{ justifyContent: 'flex-end', paddingRight: 12 }}>
        <ActionButton onClick={toggleStreaming} disabled={actionDisabled}>
          {actionLabel}
        </ActionButton>
      </ConfigRow>
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
      <CompactRow>
        <ConfigLabel>Status:</ConfigLabel>
        <ConfigValue>{streamerStateLabel}</ConfigValue>
      </CompactRow>
    </EegDevicePanel>
  )
}