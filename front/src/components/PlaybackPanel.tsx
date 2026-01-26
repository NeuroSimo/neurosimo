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

import { EegStreamContext } from 'providers/EegStreamProvider'
import { useParameters } from 'providers/ParameterProvider'
import { useSession, SessionStage } from 'providers/SessionProvider'
import { PlaybackContext } from 'providers/PlaybackProvider'

const PlaybackContainer = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH - 30}px;
  position: static;
  display: flex;
  flex-direction: column;
  gap: 4px;
`

const RecordingSelect = styled(Select)`
  margin-left: 6px;
  width: 180px;
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
  const { eegInfo } = useContext(EegStreamContext)
  const { setPlaybackBagFilename, setPlaybackIsPreprocessed } = useParameters()
  const { sessionState } = useSession()
  const { recordingsList } = useContext(PlaybackContext)

  // For playback tab - these would come from a playback context in the future
  const [playbackBagFilename, setPlaybackBagFilenameState] = useState<string>('')
  const [playbackIsPreprocessed, setPlaybackIsPreprocessedState] = useState<boolean>(false)

  const isSessionRunning = sessionState.stage !== SessionStage.STOPPED
  const isEegStreaming = eegInfo?.is_streaming || false

  const setPlaybackBagFilenameHandler = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const bagFilename = event.target.value
    setPlaybackBagFilenameState(bagFilename)
    setPlaybackBagFilename(bagFilename, () => {
      console.log('Playback bag filename set to ' + bagFilename)
    })
  }

  const setPlaybackIsPreprocessedHandler = (isPreprocessed: boolean) => {
    setPlaybackIsPreprocessedState(isPreprocessed)
    setPlaybackIsPreprocessed(isPreprocessed, () => {
      console.log('Playback is preprocessed set to ' + isPreprocessed)
    })
  }

  return (
    <PlaybackContainer isGrayedOut={isGrayedOut}>
      <ConfigRow style={{ justifyContent: 'space-between' }}>
        <ConfigLabel>Recording:</ConfigLabel>
        {recordingsList.length > 0 ? (
          <RecordingSelect onChange={setPlaybackBagFilenameHandler} value={playbackBagFilename} disabled={isSessionRunning || isEegStreaming}>
            {recordingsList.map((recordingFilename: typeof recordingsList[number], index: number) => (
              <option key={index} value={recordingFilename}>
                {recordingFilename.replace(/\.json$/, '')}
              </option>
            ))}
          </RecordingSelect>
        ) : (
          <ConfigValue>No recordings</ConfigValue>
        )}
      </ConfigRow>
      <CompactRow style={{ justifyContent: 'space-between' }}>
        <ConfigLabel>Preprocessed:</ConfigLabel>
        <SwitchWrapper>
          <ToggleSwitch
            type="flat"
            checked={playbackIsPreprocessed}
            onChange={setPlaybackIsPreprocessedHandler}
            disabled={isSessionRunning || isEegStreaming || recordingsList.length === 0}
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