import React, { useContext, useState } from 'react'
import styled from 'styled-components'

import {
  ConfigPanel,
  Select,
  ConfigRow,
  ConfigLabel,
  CONFIG_PANEL_WIDTH,
  StyledButton,
  ConfigTitle,
} from 'styles/General'

import { EegSimulatorContext } from 'providers/EegSimulatorProvider'
import { EegStreamContext } from 'providers/EegStreamProvider'
import { useSessionConfig } from 'providers/SessionConfigProvider'
import { useSession, SessionStateValue } from 'providers/SessionProvider'
import { importRecordingRos } from 'ros/eeg_simulator'

const ImportPanel = styled(ConfigPanel)`
  width: ${CONFIG_PANEL_WIDTH - 30}px;
  position: static;
  display: flex;
  flex-direction: column;
  gap: 4px;
`

const ImportSelect = styled(Select)`
  margin-left: 6px;
  width: 170px;
`

const CompactRow = styled(ConfigRow)`
  margin-bottom: 2px;
  gap: 4px;
`

const ErrorRow = styled(CompactRow)`
  margin-top: 2px;
  margin-bottom: 0;
`

const LabelSpacer = styled.div`
  width: 185px;
  flex-shrink: 0;
`

const ErrorText = styled.div`
  font-size: 11px;
  font-family: 'Roboto', 'Segoe UI', sans-serif;
  color: #c0392b;
  margin-left: 6px;
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

  return (
    <ImportPanel>
      <ConfigTitle>Import recording</ConfigTitle>
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
      {importError && (
        <ErrorRow>
          <LabelSpacer />
          <ErrorText>{importError}</ErrorText>
        </ErrorRow>
      )}
      <CompactRow style={{ justifyContent: 'flex-end', paddingRight: '10px', gap: '6px', marginTop: '6px' }}>
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
