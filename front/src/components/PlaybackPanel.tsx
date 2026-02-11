import React, { useContext, useState, useEffect } from 'react'
import styled from 'styled-components'

import { ToggleSwitch } from 'components/ToggleSwitch'
import { ExportModal, ExportDataType } from 'components/ExportModal'
import { FolderTerminalButtons } from 'components/FolderTerminalButtons'

import {
  StyledPanel,
  Select,
  ConfigRow,
  ConfigLabel,
  ConfigValue,
  NotesValue,
  CONFIG_PANEL_WIDTH,
} from 'styles/General'

import { EegStreamContext } from 'providers/EegStreamProvider'
import { useSessionConfig } from 'providers/SessionConfigProvider'
import { useSession, SessionStateValue } from 'providers/SessionProvider'
import { PlaybackContext } from 'providers/PlaybackProvider'
import { useGlobalConfig } from 'providers/GlobalConfigProvider'
import { useExporter, ExporterStateValue } from 'providers/ExporterProvider'
import { exportSessionRos } from 'ros/session'
import { getRecordingInfoRos, RecordingInfo } from 'ros/session_player'
import { formatTime, formatDateTime, formatFrequency } from 'utils/utils'

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
  margin-right: 8px;
  min-width: 115px;

  &:hover {
    background-color: ${props => props.disabled ? '#cccccc' : '#0056b3'};
  }
`

const ExportProgress = styled.span`
  font-size: 12px;
  color: #666;
  margin-right: 8px;
`

export const PlaybackPanel: React.FC<{ isGrayedOut: boolean }> = ({ isGrayedOut }) => {
  const { eegDeviceInfo } = useContext(EegStreamContext)
  const { setPlaybackBagFilename, setPlaybackIsPreprocessed } = useSessionConfig()
  const { sessionState } = useSession()
  const { recordingsList } = useContext(PlaybackContext)
  const { activeProject, locale } = useGlobalConfig()
  const { exporterState } = useExporter()

  const [selectedRecordingInfo, setSelectedRecordingInfo] = useState<RecordingInfo | null>(null)
  const [isExportModalOpen, setIsExportModalOpen] = useState(false)

  // For playback tab - these would come from a playback context in the future
  const [playbackBagFilename, setPlaybackBagFilenameState] = useState<string>('')
  const [playbackIsPreprocessed, setPlaybackIsPreprocessedState] = useState<boolean>(false)

  const isSessionRunning = sessionState.state === SessionStateValue.RUNNING
  const isEegStreaming = eegDeviceInfo?.is_streaming || false
  const isElectron = !!(window as any).electronAPI
  const isExporting = exporterState.state === ExporterStateValue.EXPORTING

  // Refetch recording info when export completes
  useEffect(() => {
    if (exporterState.state === ExporterStateValue.IDLE && playbackBagFilename) {
      getRecordingInfoRos(playbackBagFilename, (recordingInfo) => {
        if (recordingInfo) {
          setSelectedRecordingInfo(recordingInfo)
        }
      })
    }
  }, [exporterState.state, playbackBagFilename])

  // Fetch recording info when selected recording changes
  useEffect(() => {
    if (!playbackBagFilename || playbackBagFilename.trim() === '') {
      setSelectedRecordingInfo(null)
      return
    }
    getRecordingInfoRos(playbackBagFilename, (recordingInfo) => {
      if (!recordingInfo) {
        console.error('Failed to get recording info for:', playbackBagFilename)
        setSelectedRecordingInfo(null)
        return
      }
      setSelectedRecordingInfo(recordingInfo)
    })
  }, [playbackBagFilename])

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
        if (!success) {
          console.error('Export failed:', message)
        }
      }
    )
  }

  const formatFingerprintHex = (value?: number | null): string => {
    if (!value) {
      return ''
    }
    const full = value.toString(16).toUpperCase().padStart(16, '0')
    const short = full.slice(0, 6)
    return full.length > short.length ? `${short}\u2026` : short
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
      {/* <CompactRow style={{ justifyContent: 'space-between' }}>
        <ConfigLabel>Preprocessed:</ConfigLabel>
        <SwitchWrapper>
          <ToggleSwitch
            type="flat"
            checked={playbackIsPreprocessed}
            onChange={setPlaybackIsPreprocessedHandler}
            disabled={isSessionRunning || isEegStreaming || recordingsList.length === 0}
          />
        </SwitchWrapper>
      </CompactRow> */}
      {selectedRecordingInfo && (
        <>
          <CompactRow>
            <ConfigLabel>Duration:</ConfigLabel>
            <ConfigValue>{formatTime(selectedRecordingInfo.duration)}</ConfigValue>
          </CompactRow>

          <div style={{ height: '8px' }} />

          <CompactRow>
            <ConfigLabel>Sampling rate:</ConfigLabel>
            <ConfigValue>{formatFrequency(selectedRecordingInfo.sampling_frequency)}</ConfigValue>
          </CompactRow>
          <CompactRow>
            <ConfigLabel>Channels:</ConfigLabel>
          </CompactRow>
          <CompactRow>
            <ConfigLabel style={{ paddingLeft: 10 }}>EEG</ConfigLabel>
            <ConfigValue>{selectedRecordingInfo.num_eeg_channels}</ConfigValue>
          </CompactRow>
          <CompactRow>
            <ConfigLabel style={{ paddingLeft: 10 }}>EMG</ConfigLabel>
            <ConfigValue>{selectedRecordingInfo.num_emg_channels}</ConfigValue>
          </CompactRow>

          <div style={{ height: '8px' }} />

          <CompactRow>
            <ConfigLabel>Subject ID:</ConfigLabel>
            <ConfigValue>{selectedRecordingInfo.subject_id}</ConfigValue>
          </CompactRow>
          <CompactRow>
            <ConfigLabel>Time:</ConfigLabel>
            <ConfigValue>{formatDateTime(selectedRecordingInfo.start_time, locale)}</ConfigValue>
          </CompactRow>
          {selectedRecordingInfo.notes && (
            <CompactRow>
              <ConfigLabel>Notes:</ConfigLabel>
              <NotesValue>{selectedRecordingInfo.notes}</NotesValue>
            </CompactRow>
          )}

          <div style={{ height: '8px' }} />

          <CompactRow>
            <ConfigLabel>Preprocessor:</ConfigLabel>
            <ConfigValue>{selectedRecordingInfo.preprocessor_enabled ? selectedRecordingInfo.preprocessor_module : '\u2013'}</ConfigValue>
          </CompactRow>
          <CompactRow>
            <ConfigLabel>Decider:</ConfigLabel>
            <ConfigValue>{selectedRecordingInfo.decider_enabled ? selectedRecordingInfo.decider_module : '\u2013'}</ConfigValue>
          </CompactRow>
          <CompactRow>
            <ConfigLabel>Presenter:</ConfigLabel>
            <ConfigValue>{selectedRecordingInfo.presenter_enabled ? selectedRecordingInfo.presenter_module : '\u2013'}</ConfigValue>
          </CompactRow>
          <div style={{ height: '8px' }} />
          <CompactRow>
            <ConfigLabel>Fingerprints:</ConfigLabel>
          </CompactRow>
          <CompactRow>
            <ConfigLabel style={{ paddingLeft: 10 }}>Data source</ConfigLabel>
            <ConfigValue>{formatFingerprintHex(selectedRecordingInfo.data_source_fingerprint)}</ConfigValue>
          </CompactRow>
          <CompactRow>
            <ConfigLabel style={{ paddingLeft: 10 }}>Preprocessor</ConfigLabel>
            <ConfigValue>{formatFingerprintHex(selectedRecordingInfo.preprocessor_fingerprint)}</ConfigValue>
          </CompactRow>
          <CompactRow>
            <ConfigLabel style={{ paddingLeft: 10 }}>Decisions</ConfigLabel>
            <ConfigValue>{formatFingerprintHex(selectedRecordingInfo.decision_fingerprint)}</ConfigValue>
          </CompactRow>
          <div style={{ height: '8px' }} />
        </>
      )}
      <CompactRow>
        <div style={{ marginLeft: 'auto', display: 'flex', gap: '8px', alignItems: 'center' }}>
          {isExporting && (
            <ExportProgress>
              {Math.round(exporterState.progress * 100)}%
            </ExportProgress>
          )}
          <ExportButton
            disabled={recordingsList.length === 0 || isSessionRunning || isEegStreaming || isExporting}
            onClick={handleExportClick}
          >
            {isExporting ? 'Exporting...' : 'Export'}
          </ExportButton>
          <FolderTerminalButtons
            folderName={selectedRecordingInfo?.export_directory || ''}
            disabled={!selectedRecordingInfo?.exported}
            folderTitle={
              !selectedRecordingInfo?.exported
                ? "No export available"
                : isElectron
                ? "Open export folder"
                : "Only available in Electron"
            }
            terminalTitle={
              !selectedRecordingInfo?.exported
                ? "No export available"
                : isElectron
                ? "Open terminal in export folder"
                : "Only available in Electron"
            }
            size={26}
          />
        </div>
      </CompactRow>
      <ExportModal
        isOpen={isExportModalOpen}
        onClose={() => setIsExportModalOpen(false)}
        onExport={handleExport}
        recordingName={playbackBagFilename ? playbackBagFilename.replace(/\.json$/, '') : (recordingsList.length > 0 ? recordingsList[0].replace(/\.json$/, '') : 'Unknown')}
        preprocessorEnabled={selectedRecordingInfo?.preprocessor_enabled ?? true}
        deciderEnabled={selectedRecordingInfo?.decider_enabled ?? true}
        presenterEnabled={selectedRecordingInfo?.presenter_enabled ?? true}
      />
    </PlaybackContainer>
  )
}