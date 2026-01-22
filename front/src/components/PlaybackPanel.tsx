import React, { useContext, useState } from 'react'
import styled from 'styled-components'

import { ToggleSwitch } from 'components/ToggleSwitch'

import {
  StyledPanel,
  Select,
  ConfigRow,
  ConfigLabel,
  ConfigValue,
  CONFIG_PANEL_WIDTH,
} from 'styles/General'

import { PipelineContext } from 'providers/PipelineProvider'
import { EegStreamContext } from 'providers/EegStreamProvider'
import { useParameters } from 'providers/ParameterProvider'

const PlaybackContainer = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH - 30}px;
  position: static;
  display: flex;
  flex-direction: column;
  gap: 4px;
`

const ExperimentSelect = styled(Select)`
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

export const PlaybackPanel: React.FC<{ isGrayedOut: boolean }> = ({ isGrayedOut }) => {
  const { experimentState } = useContext(PipelineContext)
  const { eegInfo } = useContext(EegStreamContext)
  const { setPlaybackExperiment, setPlaybackPreprocessed } = useParameters()

  // For playback tab - these would come from a playback context in the future
  const [playbackExperiment, setPlaybackExperimentState] = useState<string>('')
  const [playbackPreprocessed, setPlaybackPreprocessedState] = useState<boolean>(false)

  const isExperimentOngoing = experimentState?.ongoing ?? false
  const isEegStreaming = eegInfo?.is_streaming || false

  const setPlaybackExperimentHandler = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const newExperiment = event.target.value
    setPlaybackExperimentState(newExperiment)
    setPlaybackExperiment(newExperiment, () => {
      console.log('Playback experiment set to ' + newExperiment)
    })
  }

  const setPlaybackPreprocessedHandler = (preprocessed: boolean) => {
    setPlaybackPreprocessedState(preprocessed)
    setPlaybackPreprocessed(preprocessed, () => {
      console.log('Playback preprocessed set to ' + preprocessed)
    })
  }

  return (
    <PlaybackContainer isGrayedOut={isGrayedOut}>
      <ConfigRow style={{ justifyContent: 'space-between' }}>
        <ConfigLabel>Experiment:</ConfigLabel>
        <ExperimentSelect onChange={setPlaybackExperimentHandler} value={playbackExperiment} disabled={isExperimentOngoing || isEegStreaming}>
          <option value="">Select experiment...</option>
          {/* TODO: Add experiment options here */}
        </ExperimentSelect>
      </ConfigRow>
      <CompactRow style={{ justifyContent: 'space-between' }}>
        <ConfigLabel>Preprocessed:</ConfigLabel>
        <SwitchWrapper>
          <ToggleSwitch
            type="flat"
            checked={playbackPreprocessed}
            onChange={setPlaybackPreprocessedHandler}
            disabled={isExperimentOngoing || isEegStreaming}
          />
        </SwitchWrapper>
      </CompactRow>
      <CompactRow>
        <ConfigLabel>Status:</ConfigLabel>
        <ConfigValue>Ready</ConfigValue>
      </CompactRow>
    </PlaybackContainer>
  )
}