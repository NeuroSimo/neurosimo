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
import { RecordingContext } from 'providers/RecordingProvider'
import { useGlobalConfig } from 'providers/GlobalConfigProvider'
import { useExporter, ExporterStateValue } from 'providers/ExporterProvider'
import { exportSessionRos } from 'ros/session'
import { getRecordingInfoRos, RecordingInfo } from 'ros/recording'
import { formatTime, formatDateTime, formatFrequency } from 'utils/utils'

const RecordingContainer = styled(StyledPanel)`
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

export const RecordingsPanel: React.FC<{ isGrayedOut: boolean }> = ({ isGrayedOut }) => {
  const { eegDeviceInfo } = useContext(EegStreamContext)
  const { setBagId, setPlayPreprocessed } = useSessionConfig()
  const { sessionState } = useSession()
  const { recordingsList } = useContext(RecordingContext)
  const { activeProject, locale } = useGlobalConfig()
  const { exporterState } = useExporter()

  const [selectedRecordingInfo, setSelectedRecordingInfo] = useState<RecordingInfo | null>(null)
  const [isExportModalOpen, setIsExportModalOpen] = useState(false)

  // For recording tab - these would come from a recording context in the future
  const [recordingBagId, setBagIdState] = useState<string>('')
  const [playPreprocessed, setPlayPreprocessedState] = useState<boolean>(false)

  const isSessionRunning = sessionState.state === SessionStateValue.RUNNING
  const isEegStreaming = eegDeviceInfo?.is_streaming || false
  const isElectron = !!(window as any).electronAPI
  const isExporting = exporterState.state === ExporterStateValue.EXPORTING

  // Refetch recording info when export completes
  useEffect(() => {
    if (exporterState.state === ExporterStateValue.IDLE && recordingBagId) {
      getRecordingInfoRos(`${recordingBagId}.json`, (recordingInfo) => {
        if (recordingInfo) {
          setSelectedRecordingInfo(recordingInfo)
        }
      })
    }
  }, [exporterState.state, recordingBagId])

  // Fetch recording info when selected recording changes
  useEffect(() => {
    if (!recordingBagId || recordingBagId.trim() === '') {
      setSelectedRecordingInfo(null)
      return
    }
    getRecordingInfoRos(`${recordingBagId}.json`, (recordingInfo) => {
      if (!recordingInfo) {
        console.error('Failed to get recording info for:', recordingBagId)
        setSelectedRecordingInfo(null)
        return
      }
      setSelectedRecordingInfo(recordingInfo)
    })
  }, [recordingBagId])

  // Set default recording when recordings become available
  useEffect(() => {
    if (recordingsList.length > 0 && !recordingBagId) {
      const defaultRecording = recordingsList[0]
      setBagIdState(defaultRecording)
      setBagId(defaultRecording, () => {
        console.log('Default recording bag id set to ' + defaultRecording)
      })
    }
  }, [recordingsList, recordingBagId, setBagId])

  const setBagIdHandler = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const bagId = event.target.value
    setBagIdState(bagId)
    setBagId(bagId, () => {
      console.log('Recording bag id set to ' + bagId)
    })
  }

  const setPlayPreprocessedHandler = (isPreprocessed: boolean) => {
    setPlayPreprocessedState(isPreprocessed)
    setPlayPreprocessed(isPreprocessed, () => {
      console.log('Replay play_preprocessed set to ' + isPreprocessed)
    })
  }

  const handleExportClick = () => {
    if (recordingsList.length > 0) {
      setIsExportModalOpen(true)
    }
  }

  const handleExport = (selectedTypes: ExportDataType[]) => {
    const recordingToExport = recordingBagId || (recordingsList.length > 0 ? recordingsList[0] : '')

    if (!recordingToExport || !activeProject) {
      console.error('No recording available or no active project')
      return
    }

    exportSessionRos(
      activeProject,
      recordingToExport,
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
    <RecordingContainer isGrayedOut={isGrayedOut}>
      <ConfigRow style={{ justifyContent: 'space-between' }}>
        <ConfigLabel>Recorded session:</ConfigLabel>
        {recordingsList.length > 0 ? (
          <RecordingSelect onChange={setBagIdHandler} value={recordingBagId} disabled={isSessionRunning || isEegStreaming}>
            {recordingsList.map((recordingId: typeof recordingsList[number], index: number) => (
              <option key={index} value={recordingId}>
                {recordingId}
              </option>
            ))}
          </RecordingSelect>
        ) : (
          <ConfigValue>No recordings</ConfigValue>
        )}
      </ConfigRow>
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
        <ConfigLabel>Replay:</ConfigLabel>
      </CompactRow>
      <CompactRow style={{ justifyContent: 'space-between' }}>
        <ConfigLabel style={{ paddingLeft: 10 }}>Play preprocessed</ConfigLabel>
        <SwitchWrapper>
          <ToggleSwitch
            type="flat"
            checked={playPreprocessed}
            onChange={setPlayPreprocessedHandler}
            disabled={isSessionRunning || isEegStreaming || recordingsList.length === 0}
          />
        </SwitchWrapper>
      </CompactRow>
      <div style={{ height: '8px' }} />
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
        recordingName={recordingBagId || (recordingsList.length > 0 ? recordingsList[0] : 'Unknown')}
        preprocessorEnabled={selectedRecordingInfo?.preprocessor_enabled ?? true}
        deciderEnabled={selectedRecordingInfo?.decider_enabled ?? true}
        presenterEnabled={selectedRecordingInfo?.presenter_enabled ?? true}
      />
    </RecordingContainer>
  )
}