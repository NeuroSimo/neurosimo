import React, { useContext, useState, useEffect } from 'react'
import styled from 'styled-components'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'
import { faInfoCircle, faExternalLinkAlt } from '@fortawesome/free-solid-svg-icons'

import { ToggleSwitch } from 'components/ToggleSwitch'
import { ExportModal, ExportDataType } from 'components/ExportModal'
import { RecordingInfoModal } from 'components/RecordingInfoModal'
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
import { getRecordingInfoRos, deleteRecordingRos, RecordingInfo } from 'ros/recording'
import { formatTime, formatDateTime, formatFrequency } from 'utils/utils'
import { DataSourceContext } from './DataSourceDisplay'

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

const InfoIcon = styled.span`
  cursor: pointer;
  color: #007bff;
  font-size: 16px;
  margin-right: 6px;
  display: inline-block;
  transition: color 0.2s;

  &:hover {
    color: #0056b3;
  }
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

const DeleteButton = styled.button<{ disabled: boolean }>`
  background-color: ${props => props.disabled ? '#cccccc' : '#dc3545'};
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
    background-color: ${props => props.disabled ? '#cccccc' : '#c82333'};
  }
`

const ExportProgress = styled.span`
  font-size: 12px;
  color: #666;
  position: absolute;
  left: -30px;
  top: 20%;
  transform: translateY(-50%);
`

const ApplyConfigButton = styled.button<{ disabled: boolean }>`
  background-color: ${props => props.disabled ? '#cccccc' : '#28a745'};
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
    background-color: ${props => props.disabled ? '#cccccc' : '#218838'};
  }
`

const DataSourceContainer = styled.div`
  display: flex;
  align-items: center;
  position: relative;
`

const DataSourceLinkIcon = styled.button`
  background: none;
  border: none;
  cursor: pointer;
  color: #007bff;
  font-size: 14px;
  position: absolute;
  left: 118px;
  display: inline-block;
  transition: color 0.2s;
  padding: 0;

  &:hover {
    color: #0056b3;
  }
`

const ButtonGrid = styled.div`
  margin-left: auto;
  margin-right: 8px;
  display: grid;
  grid-template-columns: auto auto;
  grid-template-rows: auto auto;
  gap: 8px;
  align-items: center;
  justify-items: end;
  position: relative;
`

export const RecordingsPanel: React.FC<{ isGrayedOut: boolean }> = ({ isGrayedOut }) => {
  const { eegDeviceInfo } = useContext(EegStreamContext)
  const {
    setBagId,
    setPlayPreprocessed,
    setSimulatorDataset,
    setSubjectId,
    setNotes,
    setPreprocessorEnabled,
    setPreprocessorModule,
    setDeciderEnabled,
    setDeciderModule,
    setPresenterEnabled,
    setPresenterModule
  } = useSessionConfig()
  const { sessionState } = useSession()
  const { recordingsList } = useContext(RecordingContext)
  const { activeProject, locale } = useGlobalConfig()
  const { exporterState } = useExporter()
  const dataSourceContext = useContext(DataSourceContext)

  const [selectedRecordingInfo, setSelectedRecordingInfo] = useState<RecordingInfo | null>(null)
  const [isExportModalOpen, setIsExportModalOpen] = useState(false)
  const [isInfoModalOpen, setIsInfoModalOpen] = useState(false)

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

  // Handle arrow key navigation for recording selection
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      // Only handle arrow keys when not typing in inputs and not disabled
      if (isSessionRunning || isEegStreaming || recordingsList.length === 0) return

      // Skip if user is typing in an input field
      const target = event.target as HTMLElement
      if (target.tagName === 'INPUT' || target.tagName === 'TEXTAREA') return

      const currentIndex = recordingsList.indexOf(recordingBagId)
      if (currentIndex === -1) return

      if (event.key === 'ArrowUp' && currentIndex > 0) {
        event.preventDefault()
        // Blur any currently focused element to prevent focus outline
        if (document.activeElement instanceof HTMLElement) {
          document.activeElement.blur()
        }
        const newBagId = recordingsList[currentIndex - 1]
        setBagIdState(newBagId)
        setBagId(newBagId, () => {
          console.log('Recording changed to ' + newBagId + ' via arrow key')
        })
      } else if (event.key === 'ArrowDown' && currentIndex < recordingsList.length - 1) {
        event.preventDefault()
        // Blur any currently focused element to prevent focus outline
        if (document.activeElement instanceof HTMLElement) {
          document.activeElement.blur()
        }
        const newBagId = recordingsList[currentIndex + 1]
        setBagIdState(newBagId)
        setBagId(newBagId, () => {
          console.log('Recording changed to ' + newBagId + ' via arrow key')
        })
      }
    }

    document.addEventListener('keydown', handleKeyDown)
    return () => document.removeEventListener('keydown', handleKeyDown)
  }, [recordingBagId, recordingsList, isSessionRunning, isEegStreaming, setBagId, setBagIdState])

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

  const handleDelete = () => {
    if (!recordingBagId) {
      console.error('No recording selected')
      return
    }

    const confirmDelete = window.confirm(
      `Are you sure you want to delete the recording?`
    )

    if (!confirmDelete) {
      return
    }

    deleteRecordingRos(recordingBagId, (success) => {
      if (success) {
        console.log('Recording deleted successfully')
        // Clear the current selection
        setSelectedRecordingInfo(null)
        setBagIdState('')
        setBagId('', () => {
          console.log('Recording selection cleared after deletion')
        })
        // The recordings list should be updated automatically by the context
      } else {
        console.error('Failed to delete recording')
      }
    })
  }

  const handleApplyConfig = async () => {
    if (!selectedRecordingInfo) {
      console.error('No recording selected')
      return
    }

    try {
      // Apply subject ID and notes
      await setSubjectId(selectedRecordingInfo.subject_id, () => {
        console.log('Subject ID applied from recording:', selectedRecordingInfo.subject_id)
      })

      await setNotes(selectedRecordingInfo.notes, () => {
        console.log('Notes applied from recording:', selectedRecordingInfo.notes)
      })

      // Apply preprocessor settings
      await setPreprocessorEnabled(selectedRecordingInfo.preprocessor_enabled, () => {
        console.log('Preprocessor enabled applied from recording:', selectedRecordingInfo.preprocessor_enabled)
      })

      if (selectedRecordingInfo.preprocessor_enabled && selectedRecordingInfo.preprocessor_module) {
        await setPreprocessorModule(selectedRecordingInfo.preprocessor_module, () => {
          console.log('Preprocessor module applied from recording:', selectedRecordingInfo.preprocessor_module)
        })
      }

      // Apply decider settings
      await setDeciderEnabled(selectedRecordingInfo.decider_enabled, () => {
        console.log('Decider enabled applied from recording:', selectedRecordingInfo.decider_enabled)
      })

      if (selectedRecordingInfo.decider_enabled && selectedRecordingInfo.decider_module) {
        await setDeciderModule(selectedRecordingInfo.decider_module, () => {
          console.log('Decider module applied from recording:', selectedRecordingInfo.decider_module)
        })
      }

      // Apply presenter settings
      await setPresenterEnabled(selectedRecordingInfo.presenter_enabled, () => {
        console.log('Presenter enabled applied from recording:', selectedRecordingInfo.presenter_enabled)
      })

      if (selectedRecordingInfo.presenter_enabled && selectedRecordingInfo.presenter_module) {
        await setPresenterModule(selectedRecordingInfo.presenter_module, () => {
          console.log('Presenter module applied from recording:', selectedRecordingInfo.presenter_module)
        })
      }

      console.log('Configuration applied from recording successfully')
    } catch (error) {
      console.error('Failed to apply configuration from recording:', error)
    }
  }

  const handleSimulatorLink = () => {
    if (dataSourceContext && selectedRecordingInfo?.simulator_dataset_filename) {
      // Switch to simulator tab
      dataSourceContext.setActiveTab('simulator')
      // Select the dataset from the recording
      setSimulatorDataset(selectedRecordingInfo.simulator_dataset_filename, () => {
        console.log('Switched to simulator and selected dataset:', selectedRecordingInfo.simulator_dataset_filename)
      })
    }
  }

  const handleRecordingLink = () => {
    if (selectedRecordingInfo?.replay_bag_id) {
      // Change the current recording to the replay_bag_id
      setBagIdState(selectedRecordingInfo.replay_bag_id)
      setBagId(selectedRecordingInfo.replay_bag_id, () => {
        console.log('Changed recording to replay bag:', selectedRecordingInfo.replay_bag_id)
      })
    }
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
        <ConfigLabel>Session</ConfigLabel>
        {recordingsList.length > 0 ? (
          <div style={{ display: 'flex', alignItems: 'center' }}>
            <InfoIcon onClick={() => setIsInfoModalOpen(true)} title="View recording details">
              <FontAwesomeIcon icon={faInfoCircle} />
            </InfoIcon>
            <RecordingSelect onChange={setBagIdHandler} value={recordingBagId} disabled={isSessionRunning || isEegStreaming}>
              {recordingsList.map((recordingId: typeof recordingsList[number], index: number) => (
                <option key={index} value={recordingId}>
                  {recordingId}
                </option>
              ))}
            </RecordingSelect>
          </div>
        ) : (
          <ConfigValue>No recordings</ConfigValue>
        )}
      </ConfigRow>
      {selectedRecordingInfo && (
        <>
          <CompactRow>
            <ConfigLabel>Duration</ConfigLabel>
            <ConfigValue>{formatTime(selectedRecordingInfo.duration)}</ConfigValue>
          </CompactRow>

          <div style={{ height: '8px' }} />

          <CompactRow>
            <ConfigLabel>Sampling rate</ConfigLabel>
            <ConfigValue>{formatFrequency(selectedRecordingInfo.sampling_frequency)}</ConfigValue>
          </CompactRow>
          <CompactRow>
            <ConfigLabel>Channels</ConfigLabel>
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
            <ConfigLabel>Subject ID</ConfigLabel>
            <ConfigValue>{selectedRecordingInfo.subject_id}</ConfigValue>
          </CompactRow>
          <CompactRow>
            <ConfigLabel>Data Source</ConfigLabel>
            <DataSourceContainer>
              {selectedRecordingInfo.data_source === 'simulator' && selectedRecordingInfo.simulator_dataset_filename ? (
                <>
                  <DataSourceLinkIcon onClick={handleSimulatorLink} title="Move to simulator">
                    <FontAwesomeIcon icon={faExternalLinkAlt} />
                  </DataSourceLinkIcon>
                  <ConfigValue>Simulator</ConfigValue>
                </>
              ) : selectedRecordingInfo.data_source === 'recording' && selectedRecordingInfo.replay_bag_id ? (
                <>
                  <DataSourceLinkIcon onClick={handleRecordingLink} title="Move to recording">
                    <FontAwesomeIcon icon={faExternalLinkAlt} />
                  </DataSourceLinkIcon>
                  <ConfigValue>Recording</ConfigValue>
                </>
              ) : selectedRecordingInfo.data_source === 'eeg_device' ? (
                <ConfigValue>EEG Device</ConfigValue>
              ) : null}
            </DataSourceContainer>
          </CompactRow>
          <CompactRow>
            <ConfigLabel>Time</ConfigLabel>
            <ConfigValue>{formatDateTime(selectedRecordingInfo.start_time, locale)}</ConfigValue>
          </CompactRow>
          <CompactRow>
            <ConfigLabel>Notes</ConfigLabel>
            <NotesValue>{selectedRecordingInfo.notes || '\u2013'}</NotesValue>
          </CompactRow>

          <div style={{ height: '8px' }} />

          <CompactRow>
            <ConfigLabel>Preprocessor</ConfigLabel>
            <ConfigValue>{selectedRecordingInfo.preprocessor_enabled ? selectedRecordingInfo.preprocessor_module : '\u2013'}</ConfigValue>
          </CompactRow>
          <CompactRow>
            <ConfigLabel>Decider</ConfigLabel>
            <ConfigValue>{selectedRecordingInfo.decider_enabled ? selectedRecordingInfo.decider_module : '\u2013'}</ConfigValue>
          </CompactRow>
          <CompactRow>
            <ConfigLabel>Presenter</ConfigLabel>
            <ConfigValue>{selectedRecordingInfo.presenter_enabled ? selectedRecordingInfo.presenter_module : '\u2013'}</ConfigValue>
          </CompactRow>
          <div style={{ height: '8px' }} />
          <CompactRow>
            <ConfigLabel>Fingerprints</ConfigLabel>
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
        <ConfigLabel>Replay</ConfigLabel>
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
        <ButtonGrid>
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
          <div style={{ justifySelf: 'center' }}>
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
          <ApplyConfigButton
            disabled={!selectedRecordingInfo || isSessionRunning || isEegStreaming}
            onClick={handleApplyConfig}
          >
            Apply config
          </ApplyConfigButton>
          <DeleteButton
            disabled={!recordingBagId || isSessionRunning || isEegStreaming}
            onClick={handleDelete}
          >
            Delete
          </DeleteButton>
        </ButtonGrid>
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
      <RecordingInfoModal
        isOpen={isInfoModalOpen}
        onClose={() => setIsInfoModalOpen(false)}
        recordingInfo={selectedRecordingInfo}
        recordingName={recordingBagId || 'Unknown'}
      />
    </RecordingContainer>
  )
}