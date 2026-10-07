import React, { useContext, useEffect, useState } from 'react'
import styled from 'styled-components'

import {
  ConfigPanel,
  Select,
  ConfigRow,
  ConfigLabel,
  CONFIG_PANEL_WIDTH,
  StyledButton,
  ConfigTitle,
  palette,
} from 'styles/General'

import { EegSimulatorContext } from 'providers/EegSimulatorProvider'
import { EegStreamContext } from 'providers/EegStreamProvider'
import { useSessionConfig } from 'providers/SessionConfigProvider'
import { useSession, SessionStateValue } from 'providers/SessionProvider'
import { importRecordingRos } from 'ros/eeg_simulator'

const ImportPanel = styled(ConfigPanel)`
  width: ${CONFIG_PANEL_WIDTH}px;
  position: static;
  display: flex;
  flex-direction: column;
  gap: 2px;
  padding: 10px 0 8px 0;
`

const ImportSelect = styled(Select)`
  margin-left: 6px;
  flex-shrink: 0;
`

const CompactRow = styled(ConfigRow)`
  margin-bottom: 2px;
  gap: 4px;
`

/* The header line doubles as the import status line, so transient status needs no space of its own
   and the section keeps the same height in every state. */
const HeaderRow = styled.div`
  display: flex;
  align-items: baseline;
  justify-content: space-between;
  gap: 8px;
  margin-bottom: 10px;
  padding-right: 17px;
`

const HeaderTitle = styled(ConfigTitle)`
  margin: 0;
  flex-shrink: 0;
`

const HeaderStatus = styled.span<{ $error: boolean }>`
  min-width: 0;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
  font-size: 11px;
  color: ${props => props.$error ? palette.red : palette.textMuted};
`

export const ImportRecordingPanel: React.FC = () => {
  const { externalRecordingsList } = useContext(EegSimulatorContext)
  const { eegDeviceInfo } = useContext(EegStreamContext)
  const { setSimulatorDataset } = useSessionConfig()
  const { sessionState } = useSession()

  const [importFile, setImportFile] = useState<string>('')
  const [importError, setImportError] = useState<string>('')
  const [isImporting, setIsImporting] = useState(false)

  const isSessionRunning = sessionState.state === SessionStateValue.RUNNING
  const isEegStreaming = eegDeviceInfo?.is_streaming || false

  const selectedFile = importFile || externalRecordingsList[0] || ''

  useEffect(() => {
    setImportFile('')
  }, [externalRecordingsList])

  useEffect(() => {
    setImportError('')
  }, [selectedFile, sessionState.state])

  const confirmImport = () => {
    if (!selectedFile) return
    setIsImporting(true)
    setImportError('')
    importRecordingRos(selectedFile, (result) => {
      setIsImporting(false)
      if (!result || !result.success) {
        setImportError(result?.message || 'Import failed')
      } else {
        setSimulatorDataset(result.dataset_filename, () => {
          console.log('Dataset set to imported file: ' + result.dataset_filename)
        })
      }
    })
  }

  const statusText = isImporting ? 'Importing…' : importError

  return (
    <ImportPanel>
      <HeaderRow>
        <HeaderTitle>Import recording</HeaderTitle>
        {statusText && (
          <HeaderStatus $error={!isImporting} title={statusText}>{statusText}</HeaderStatus>
        )}
      </HeaderRow>
      <CompactRow>
        <ConfigLabel>File</ConfigLabel>
        <ImportSelect
          value={selectedFile}
          onChange={(e) => setImportFile(e.target.value)}
          disabled={isImporting || externalRecordingsList.length === 0}
        >
          {externalRecordingsList.length === 0
            ? <option value=''>—</option>
            : externalRecordingsList.map((filename, index) => (
                <option key={index} value={filename}>{filename}</option>
              ))
          }
        </ImportSelect>
      </CompactRow>
      <CompactRow style={{ justifyContent: 'flex-end', paddingRight: '17px', gap: '6px', marginTop: '6px' }}>
        <StyledButton
          onClick={confirmImport}
          disabled={isImporting || isSessionRunning || isEegStreaming || externalRecordingsList.length === 0}
        >
          {isImporting ? 'Importing…' : 'Import'}
        </StyledButton>
      </CompactRow>
    </ImportPanel>
  )
}
