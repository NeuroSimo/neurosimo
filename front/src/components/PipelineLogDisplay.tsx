import React, { useContext, useRef, useEffect, useState } from 'react'
import styled from 'styled-components'

import { PIPELINE_LOG_OFFSET_FROM_TOP, palette, selectChevron } from 'styles/General'

import { LogContext, LogMessage, LogLevel, LogPhase, ProcessingPath } from 'providers/LogProvider'
import { StickyBottomScrollContainer } from 'components/StickyBottomScrollContainer'

type LogSource = 'preprocessor' | 'decider' | 'presenter'

const PIPELINE_LOG_WIDTH = 983
const PIPELINE_LOG_TOOLBAR_HEIGHT = 34

const PipelineLogPanelTitle = styled.div`
  width: ${PIPELINE_LOG_WIDTH}px;
  height: ${PIPELINE_LOG_TOOLBAR_HEIGHT}px;
  box-sizing: border-box;
  padding: 0 8px 0 12px;
  position: fixed;
  top: ${PIPELINE_LOG_OFFSET_FROM_TOP}px;
  right: 0;
  z-index: 1001;
  text-align: left;
  font-size: 11px;
  font-weight: 600;
  letter-spacing: 0.06em;
  text-transform: uppercase;
  color: ${palette.textSecondary};
  background-color: ${palette.surface};
  border-top: 1px solid ${palette.border};
  border-left: 1px solid ${palette.border};
  border-bottom: 1px solid ${palette.border};
  display: flex;
  justify-content: space-between;
  align-items: center;
`

const TitleGroup = styled.div`
  display: flex;
  gap: 10px;
  align-items: center;
`

const LogSourceSelect = styled.select`
  appearance: none;
  -webkit-appearance: none;
  height: 24px;
  box-sizing: border-box;
  background-color: ${palette.surfaceRaised};
  background-image: ${selectChevron};
  background-repeat: no-repeat;
  background-position: right 8px center;
  color: ${palette.text};
  border: 1px solid ${palette.borderStrong};
  border-radius: 3px;
  padding: 0 24px 0 8px;
  font-size: 12px;
  font-weight: 500;
  letter-spacing: normal;
  text-transform: none;
  cursor: pointer;
  transition: border-color 0.2s;

  &:hover {
    border-color: #4a4f57;
  }

  &:focus {
    outline: none;
    border-color: ${palette.blue};
  }
`

const ButtonGroup = styled.div`
  display: flex;
  gap: 6px;
`

const LogButton = styled.button`
  height: 24px;
  background-color: ${palette.surfaceRaised};
  color: ${palette.text};
  border: 1px solid ${palette.borderStrong};
  border-radius: 3px;
  padding: 0 10px;
  font-size: 12px;
  font-weight: 500;
  letter-spacing: normal;
  text-transform: none;
  cursor: pointer;
  transition: background-color 0.2s;

  &:hover {
    background-color: ${palette.surfaceHover};
  }

  &:active {
    background-color: ${palette.borderStrong};
  }

  &:disabled {
    background-color: ${palette.surface};
    color: ${palette.textDim};
    border-color: ${palette.border};
    cursor: not-allowed;
  }
`

const PipelineLogPanel = styled.div`
  width: ${PIPELINE_LOG_WIDTH}px;
  box-sizing: border-box;
  position: fixed;
  top: ${PIPELINE_LOG_OFFSET_FROM_TOP + PIPELINE_LOG_TOOLBAR_HEIGHT}px;
  bottom: 0;
  right: 0;
  z-index: 1000;
  padding: 0;
  border-left: 1px solid ${palette.border};
  background-color: ${palette.console};
  display: flex;
  flex-direction: column;
`

const LogContainer = styled(StickyBottomScrollContainer)`
  flex: 1;
  overflow-y: auto;
  background-color: ${palette.console};
  padding: 6px 10px;
  font-family: ${palette.monoFont};
  font-size: 12px;
  line-height: 1.4;
  white-space: pre-wrap;
  word-wrap: break-word;
  user-select: text;
  -webkit-user-select: text;
  -moz-user-select: text;
  -ms-user-select: text;
`

const LogEntry = styled.div`
  margin-bottom: 1px;
  color: #c9cdd2;
  display: grid;
  grid-template-columns: 65px 1fr;
  gap: 0;
`

const Timestamp = styled.span<{ $phase: number; $level: number }>`
  color: ${props => {
    // Check error level first so errors during init/finalize show correctly
    if (props.$level === 2) return '#ff8a85'  // ERROR - red
    if (props.$level === 1) return '#e6c662'  // WARNING - yellow
    if (props.$phase === 0) return palette.orange  // INITIALIZATION - orange
    if (props.$phase === 2) return '#c0c4ca'  // FINALIZATION - light gray
    return palette.textMuted  // INFO - muted gray
  }};
  font-weight: 600;
  text-align: right;
  background-color: ${props => {
    // Check error level first so errors during init/finalize show correctly
    if (props.$level === 2) return 'rgba(207, 81, 73, 0.22)'  // ERROR - red
    if (props.$level === 1) return 'rgba(216, 180, 69, 0.16)'  // WARNING - yellow
    if (props.$phase === 0) return 'rgba(229, 149, 74, 0.16)'  // INITIALIZATION - orange
    if (props.$phase === 2) return palette.border  // FINALIZATION - gray
    return palette.surface  // INFO - dark gray
  }};
  padding: 1px 6px;
  border-right: 2px solid ${props => {
    // Check error level first so errors during init/finalize show correctly
    if (props.$level === 2) return palette.red  // ERROR - red
    if (props.$level === 1) return '#a88a2c'  // WARNING - darker yellow
    if (props.$phase === 0) return '#a8692f'  // INITIALIZATION - darker orange
    if (props.$phase === 2) return '#545a63'  // FINALIZATION - darker gray
    return palette.borderStrong  // INFO - gray
  }};
`

const LogText = styled.span<{ $processingPath: number }>`
  padding: 1px 10px;
  background-color: ${props => {
    if (props.$processingPath === ProcessingPath.PULSE) return 'rgba(74, 138, 212, 0.10)'  // PULSE - blue tint
    if (props.$processingPath === ProcessingPath.EVENT) return 'rgba(79, 191, 139, 0.10)'  // EVENT - green tint
    if (props.$processingPath === ProcessingPath.PREPARE_TRIAL) return 'rgba(155, 89, 182, 0.12)'  // PREPARE_TRIAL - purple tint
    return 'transparent'  // UNDETERMINED and PERIODIC - no background
  }};
  border-left: ${props => {
    if (props.$processingPath === ProcessingPath.PULSE) return `2px solid ${palette.blue}`  // PULSE - blue border
    if (props.$processingPath === ProcessingPath.EVENT) return `2px solid ${palette.green}`  // EVENT - green border
    if (props.$processingPath === ProcessingPath.PREPARE_TRIAL) return '2px solid #9b59b6'  // PREPARE_TRIAL - purple border
    return 'none'  // UNDETERMINED and PERIODIC - no border
  }};
`

export const PipelineLogDisplay: React.FC = () => {
  const { preprocessorLogs, deciderLogs, presenterLogs, clearAllLogs } = useContext(LogContext)
  const [selectedSource, setSelectedSource] = useState<LogSource>('decider')
  const prevLogLengthsRef = useRef({ preprocessor: 0, decider: 0, presenter: 0 })

  // Get the currently selected logs
  const currentLogs =
    selectedSource === 'preprocessor' ? preprocessorLogs :
    selectedSource === 'decider' ? deciderLogs :
    presenterLogs

  // Auto-switch to tab when new error messages arrive
  useEffect(() => {
    const checkForNewErrors = (logs: LogMessage[], source: LogSource, prevLength: number) => {
      const newLogs = logs.slice(prevLength)
      const hasNewErrors = newLogs.some(log => log.level === LogLevel.ERROR)
      if (hasNewErrors && selectedSource !== source) {
        setSelectedSource(source)
      }
    }

    checkForNewErrors(preprocessorLogs, 'preprocessor', prevLogLengthsRef.current.preprocessor)
    checkForNewErrors(deciderLogs, 'decider', prevLogLengthsRef.current.decider)
    checkForNewErrors(presenterLogs, 'presenter', prevLogLengthsRef.current.presenter)

    // Update previous lengths
    prevLogLengthsRef.current = {
      preprocessor: preprocessorLogs.length,
      decider: deciderLogs.length,
      presenter: presenterLogs.length,
    }
  }, [preprocessorLogs, deciderLogs, presenterLogs, selectedSource])

  const getTimestampLabel = (log: LogMessage): string => {
    // Check error level first so errors during init/finalize still show as "Error"
    if (log.level === LogLevel.ERROR) return 'Error'
    if (log.phase === LogPhase.INITIALIZATION) return 'Init'
    if (log.phase === LogPhase.FINALIZATION) return 'Final'
    // If the sample time is NaN in the backend, it will be null here. This shouldn't happen unless there is a bug,
    // but if it does, we'll show a question mark.
    if (log.sample_time == null) return '?'
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

  const handleClearAllLogs = () => {
    clearAllLogs()
    // Reset error tracking when clearing logs
    prevLogLengthsRef.current = { preprocessor: 0, decider: 0, presenter: 0 }
  }

  const handleSourceChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const newSource = event.target.value as LogSource
    setSelectedSource(newSource)
    // Reset error tracking when manually switching tabs
    prevLogLengthsRef.current = {
      preprocessor: preprocessorLogs.length,
      decider: deciderLogs.length,
      presenter: presenterLogs.length,
    }
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
          <LogButton onClick={handleClearAllLogs}>Clear All</LogButton>
        </ButtonGroup>
      </PipelineLogPanelTitle>
      <PipelineLogPanel>
        <LogContainer contentDependency={currentLogs} resetScrollDependency={selectedSource}>
          {currentLogs.length === 0 ? (
            <LogEntry style={{ color: palette.textDim, fontStyle: 'italic', display: 'block' }}>
              No logs...
            </LogEntry>
          ) : (
            currentLogs.map((log: LogMessage, index: number) => (
              <LogEntry key={index}>
                <Timestamp $phase={log.phase} $level={log.level}>
                  {getTimestampLabel(log)}
                </Timestamp>
                <LogText $processingPath={log.processing_path}>{log.message}</LogText>
              </LogEntry>
            ))
          )}
        </LogContainer>
      </PipelineLogPanel>
    </>
  )
}


