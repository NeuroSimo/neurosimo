import React, { useContext, useState, useEffect } from 'react'
import styled from 'styled-components'

import { ToggleSwitch } from 'components/ToggleSwitch'
import { ExportModal, ExportDataType } from 'components/ExportModal'

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
import { ProjectContext } from 'providers/ProjectProvider'
import { exportSessionRos } from 'ros/session'

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

const ExportButton = styled.button<{ disabled: boolean }>`
  background-color: ${props => props.disabled ? '#cccccc' : '#007bff'};
  color: ${props => props.disabled ? '#666666' : 'white'};
  border: none;
  border-radius: 4px;
  padding: 6px 12px;
  cursor: ${props => props.disabled ? 'not-allowed' : 'pointer'};
  font-size: 14px;
  font-weight: 500;
  margin-left: auto;

  &:hover {
    background-color: ${props => props.disabled ? '#cccccc' : '#0056b3'};
  }
`

export const PlaybackPanel: React.FC<{ isGrayedOut: boolean }> = ({ isGrayedOut }) => {
  const { eegInfo } = useContext(EegStreamContext)
  const { setPlaybackBagFilename, setPlaybackIsPreprocessed } = useParameters()
  const { sessionState } = useSession()
  const { recordingsList } = useContext(PlaybackContext)
  const { activeProject } = useContext(ProjectContext)
  const [isExportModalOpen, setIsExportModalOpen] = useState(false)

  // For playback tab - these would come from a playback context in the future
  const [playbackBagFilename, setPlaybackBagFilenameState] = useState<string>('')
  const [playbackIsPreprocessed, setPlaybackIsPreprocessedState] = useState<boolean>(false)

  const isSessionRunning = sessionState.stage !== SessionStage.STOPPED
  const isEegStreaming = eegInfo?.is_streaming || false

  // Set default recording when recordings become available
  useEffect(() => {
    if (recordingsList.length > 0 && !playbackBagFilename) {
      const defaultRecording = recordingsList[0]
      setPlaybackBagFilenameState(defaultRecording)
      setPlaybackBagFilename(defaultRecording, () => {
        console.log('Default playback bag filename set to ' + defaultRecording)
      })
    }
  }, [recordingsList, playbackBagFilename, setPlaybackBagFilename])

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

  const handleExportClick = () => {
    if (recordingsList.length > 0) {
      setIsExportModalOpen(true)
    }
  }

  const handleExport = (selectedTypes: ExportDataType[]) => {
    const recordingToExport = playbackBagFilename || (recordingsList.length > 0 ? recordingsList[0] : '')

    if (!recordingToExport || !activeProject) {
      console.error('No recording available or no active project')
      return
    }

    // Remove .json extension for the backend (expects directory name)
    const recordingName = recordingToExport.replace(/\.json$/, '')

    exportSessionRos(
      activeProject,
      recordingName,
      selectedTypes,
      (success, message) => {
        if (success) {
          console.log('Export completed successfully')
        } else {
          console.error('Export failed:', message)
        }
      }
    )
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
      <CompactRow>
        <ExportButton
          disabled={recordingsList.length === 0 || isSessionRunning || isEegStreaming}
          onClick={handleExportClick}
        >
          Export
        </ExportButton>
      </CompactRow>
      <ExportModal
        isOpen={isExportModalOpen}
        onClose={() => setIsExportModalOpen(false)}
        onExport={handleExport}
        recordingName={playbackBagFilename ? playbackBagFilename.replace(/\.json$/, '') : (recordingsList.length > 0 ? recordingsList[0].replace(/\.json$/, '') : 'Unknown')}
      />
    </PlaybackContainer>
  )
}