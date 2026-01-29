import React, { useContext, useRef, useEffect, useState } from 'react'
import styled from 'styled-components'

import { StyledPanel, DASHBOARD_PANEL_OFFSET_FROM_TOP } from 'styles/General'

import { LogContext, LogMessage, LogLevel } from 'providers/LogProvider'

type LogSource = 'preprocessor' | 'decider' | 'presenter'

const PipelineLogPanelTitle = styled.div`
  width: 735px;
  position: fixed;
  top: 610px;
  right: 27px;
  z-index: 1001;
  text-align: left;
  font-size: 12px;
  font-weight: bold;
  display: flex;
  justify-content: space-between;
  align-items: center;
`

const TitleGroup = styled.div`
  display: flex;
  gap: 7px;
  align-items: center;
`

const LogSourceSelect = styled.select`
  background-color: #f5f5f5;
  border: 1.3px solid #d46c0b;
  border-radius: 3px;
  padding: 3.5px 8px;
  font-size: 11px;
  font-weight: bold;
  cursor: pointer;
  transition: border-color 0.2s, background-color 0.2s;

  &:hover {
    background-color: #fff;
    border-color: #b85a09;
  }

  &:focus {
    outline: none;
    border-color: #9a4a07;
  }
`

const ButtonGroup = styled.div`
  display: flex;
  gap: 5px;
`

const LogButton = styled.button`
  background-color: #d46c0b;
  color: white;
  border: none;
  border-radius: 3px;
  padding: 3.5px 8px;
  font-size: 10px;
  cursor: pointer;
  transition: background-color 0.2s;

  &:hover {
    background-color: #b85a09;
  }

  &:active {
    background-color: #9a4a07;
  }

  &:disabled {
    background-color: #ccc;
    cursor: not-allowed;
  }
`

const PipelineLogPanel = styled(StyledPanel)`
  width: 733px;
  height: 230px;
  position: fixed;
  top: 650px;
  right: 10px;
  z-index: 1000;
  padding: 9px;
  display: flex;
  flex-direction: column;
`

const LogContainer = styled.div`
  flex: 1;
  overflow-y: auto;
  background-color: #f9f9f9;
  border: 1px solid #ddd;
  border-radius: 3px;
  padding: 6px;
  font-family: 'Courier New', monospace;
  font-size: 9px;
  line-height: 1.25;
  white-space: pre-wrap;
  word-wrap: break-word;
  user-select: text;
  -webkit-user-select: text;
  -moz-user-select: text;
  -ms-user-select: text;
`

const LogEntry = styled.div`
  margin-bottom: 3px;
  color: #333;
  display: grid;
  grid-template-columns: 50px 1fr;
  gap: 0;
`

const Timestamp = styled.span<{ $isInit: boolean; $level: number }>`
  color: ${props => {
    if (props.$isInit) return '#000'  // INIT - black
    if (props.$level === 2) return '#fff'  // ERROR - white
    return '#555'  // INFO/WARNING - dark gray
  }};
  font-weight: bold;
  text-align: right;
  background-color: ${props => {
    if (props.$isInit) return '#d46c0b'  // INIT - orange
    if (props.$level === 2) return '#dc3545'  // ERROR - red
    if (props.$level === 1) return '#ffc107'  // WARNING - yellow
    return '#e8e8e8'  // INFO - light gray
  }};
  padding: 2px 3px;
  border-right: 2px solid ${props => {
    if (props.$isInit) return '#b85a09'  // INIT - darker orange
    if (props.$level === 2) return '#c82333'  // ERROR - darker red
    if (props.$level === 1) return '#e0a800'  // WARNING - darker yellow
    return '#ccc'  // INFO - gray
  }};
`

const LogText = styled.span`
  padding: 2px 10px;
`

export const PipelineLogDisplay: React.FC = () => {
  const { preprocessorLogs, deciderLogs, presenterLogs, clearAllLogs } = useContext(LogContext)
  const [selectedSource, setSelectedSource] = useState<LogSource>('decider')
  const logContainerRef = useRef<HTMLDivElement>(null)

  // Get the currently selected logs
  const currentLogs = 
    selectedSource === 'preprocessor' ? preprocessorLogs :
    selectedSource === 'decider' ? deciderLogs :
    presenterLogs

  // Auto-scroll to bottom when new logs arrive
  useEffect(() => {
    if (logContainerRef.current) {
      logContainerRef.current.scrollTop = logContainerRef.current.scrollHeight
    }
  }, [currentLogs])

  const getTimestampLabel = (log: LogMessage): string => {
    if (log.is_initialization) return 'Init'
    if (log.level === LogLevel.ERROR) return 'Error'
    return log.sample_time.toFixed(3)
  }

  const handleCopyLogs = async () => {
    const logsText = currentLogs
      .map((log: LogMessage) => `${getTimestampLabel(log)} ${log.message}`)
      .join('\n')
    try {
      await navigator.clipboard.writeText(logsText)
    } catch (err) {
      console.error('Failed to copy logs:', err)
    }
  }

  const handleSourceChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    setSelectedSource(event.target.value as LogSource)
  }

  return (
    <>
      <PipelineLogPanelTitle>
        <TitleGroup>
          <span>Pipeline logs:</span>
          <LogSourceSelect value={selectedSource} onChange={handleSourceChange}>
            <option value="preprocessor">Preprocessor</option>
            <option value="decider">Decider</option>
            <option value="presenter">Presenter</option>
          </LogSourceSelect>
        </TitleGroup>
        <ButtonGroup>
          <LogButton onClick={handleCopyLogs} disabled={currentLogs.length === 0}>
            Copy
          </LogButton>
          <LogButton onClick={clearAllLogs}>Clear All</LogButton>
        </ButtonGroup>
      </PipelineLogPanelTitle>
      <PipelineLogPanel>
        <LogContainer ref={logContainerRef}>
          {currentLogs.length === 0 ? (
            <LogEntry style={{ color: '#999', fontStyle: 'italic', display: 'block' }}>
              No logs...
            </LogEntry>
          ) : (
            currentLogs.map((log: LogMessage, index: number) => (
              <LogEntry key={index}>
                <Timestamp $isInit={log.is_initialization} $level={log.level}>
                  {getTimestampLabel(log)}
                </Timestamp>
                <LogText>{log.message}</LogText>
              </LogEntry>
            ))
          )}
        </LogContainer>
      </PipelineLogPanel>
    </>
  )
}


