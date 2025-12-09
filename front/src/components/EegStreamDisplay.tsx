import React, { useContext, useEffect, useState } from 'react'
import styled from 'styled-components'

import { StyledPanel, StateRow, StateTitle, IndentedStateTitle, StateValue } from 'styles/General'

import { EegStreamContext } from 'providers/EegStreamProvider'
import { HealthcheckContext, HealthcheckStatus } from 'providers/HealthcheckProvider'

import { formatFrequency } from 'utils/utils'

const EegPanel = styled(StyledPanel)`
  width: 185px;
  height: 80px;
  position: fixed;
  top: 148px;
  right: 4px;
  z-index: 1000;
`

export const EegStreamDisplay: React.FC = () => {
  const { eegInfo } = useContext(EegStreamContext)
  const { eegHealthcheck } = useContext(HealthcheckContext)

  const eegHealthcheckOk = eegHealthcheck?.status.value === HealthcheckStatus.READY
  const numOfEegChannels = eegInfo?.num_eeg_channels || 0
  const numOfEmgChannels = eegInfo?.num_emg_channels || 0

  return (
    <EegPanel isGrayedOut={!eegHealthcheckOk}>
      <StateRow>
        <StateTitle>Sampling rate:</StateTitle>
        <StateValue>{formatFrequency(eegInfo?.sampling_frequency)}</StateValue>
      </StateRow>
      <br />
      <StateRow>
        <StateTitle>Channels:</StateTitle>
      </StateRow>
      <StateRow>
        <IndentedStateTitle>EEG</IndentedStateTitle>
        <StateValue>{numOfEegChannels > 0 ? numOfEegChannels : '\u2013'}</StateValue>
      </StateRow>
      <StateRow>
        <IndentedStateTitle>EMG</IndentedStateTitle>
        <StateValue>{numOfEmgChannels > 0 ? numOfEmgChannels : '\u2013'}</StateValue>
      </StateRow>
    </EegPanel>
  )
}
