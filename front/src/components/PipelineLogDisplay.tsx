import React, { useContext, useRef, useEffect, useState } from 'react'
import styled from 'styled-components'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'
import { faCheck, faCopy, faTrashAlt } from '@fortawesome/free-solid-svg-icons'

import {
  PIPELINE_LOG_BODY_HEIGHT,
  PIPELINE_LOG_OFFSET_FROM_TOP,
  PIPELINE_LOG_WIDTH,
  palette,
  selectChevron,
} from 'styles/General'

import { LogContext, LogMessage, LogLevel, LogPhase, ProcessingPath } from 'providers/LogProvider'
import { StickyBottomScrollContainer } from 'components/StickyBottomScrollContainer'

type LogSource = 'preprocessor' | 'decider' | 'presenter'

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
  gap: 14px;
  align-items: center;
`

/* Compact filter for the log source, kept visually lighter than the form selects. */
const LogSourceSelect = styled.select`
  appearance: none;
  -webkit-appearance: none;
  height: 22px;
  box-sizing: border-box;
  background-color: transparent;
  background-image: ${selectChevron};
  background-repeat: no-repeat;
  background-position: right 7px center;
  color: ${palette.textSecondary};
  border: 1px solid ${palette.border};
  border-radius: 3px;
  padding: 0 22px 0 7px;
  font-size: 11.5px;
  font-weight: 500;
  letter-spacing: normal;
  text-transform: none;
  cursor: pointer;
  transition: border-color 0.2s, color 0.2s;

  &:hover {
    border-color: ${palette.borderStrong};
    color: ${palette.text};
  }

  &:focus {
    outline: none;
    border-color: ${palette.accent};
    color: ${palette.text};
  }

  option {
    background-color: ${palette.surfaceRaised};
    color: ${palette.text};
  }
`

const ButtonGroup = styled.div`
  display: flex;
  gap: 2px;
  margin-left: -4px;
`

const LogIconButton = styled.button<{ $destructive?: boolean }>`
  width: 24px;
  height: 24px;
  display: flex;
  align-items: center;
  justify-content: center;
  background: none;
  border: none;
  border-radius: 3px;
  padding: 0;
  color: ${palette.icon};
  font-size: 12px;
  cursor: pointer;
  transition: color 0.2s, background-color 0.2s;

  &:hover:not(:disabled),
  &:focus-visible {
    color: ${props => props.$destructive ? palette.red : palette.iconHover};
    background-color: ${palette.surfaceHover};
  }

  &:focus {
    outline: none;
  }

  &:focus-visible {
    box-shadow: 0 0 0 1px ${palette.accent};
  }

  &:disabled {
    opacity: 0.4;
    cursor: not-allowed;
  }
`

const PipelineLogPanel = styled.div`
  width: ${PIPELINE_LOG_WIDTH}px;
  box-sizing: border-box;
  position: fixed;
  top: ${PIPELINE_LOG_OFFSET_FROM_TOP + PIPELINE_LOG_TOOLBAR_HEIGHT}px;
  height: ${PIPELINE_LOG_BODY_HEIGHT}px;
  right: 0;
  z-index: 1000;
  padding: 0;
  border-left: 1px solid ${palette.border};
  border-bottom: 1px solid ${palette.border};
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
  const [copied, setCopied] = useState(false)
  const copiedTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null)

  useEffect(() => () => {
    if (copiedTimeoutRef.current) clearTimeout(copiedTimeoutRef.current)
  }, [])

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
      setCopied(true)
      if (copiedTimeoutRef.current) clearTimeout(copiedTimeoutRef.current)
      copiedTimeoutRef.current = setTimeout(() => setCopied(false), 1500)
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
          <span>Logs</span>
          <LogSourceSelect value={selectedSource} onChange={handleSourceChange} aria-label="Log source">
            <option value="preprocessor">Preprocessor</option>
            <option value="decider">Decider</option>
            <option value="presenter">Presenter</option>
          </LogSourceSelect>
          <ButtonGroup>
            <LogIconButton
              onClick={handleCopyLogs}
              disabled={currentLogs.length === 0}
              title="Copy logs"
              aria-label="Copy logs"
            >
              {copied
                ? <FontAwesomeIcon icon={faCheck} style={{ color: palette.green }} />
                : <FontAwesomeIcon icon={faCopy} />}
            </LogIconButton>
            <LogIconButton onClick={handleClearAllLogs} title="Clear logs" aria-label="Clear logs" $destructive>
              <FontAwesomeIcon icon={faTrashAlt} />
            </LogIconButton>
          </ButtonGroup>
        </TitleGroup>
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


