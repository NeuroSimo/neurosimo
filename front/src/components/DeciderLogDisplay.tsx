import React, { useContext, useRef, useEffect } from 'react'
import styled from 'styled-components'

import { StyledPanel } from 'styles/General'

import { PipelineContext } from 'providers/PipelineProvider'

const DeciderLogPanelTitle = styled.div`
  width: 680px;
  position: fixed;
  top: 686px;
  left: 55px;
  z-index: 1001;
  text-align: left;
  font-size: 20px;
  font-weight: bold;
  display: flex;
  justify-content: space-between;
  align-items: center;
`

const ClearButton = styled.button`
  background-color: #d46c0b;
  color: white;
  border: none;
  border-radius: 4px;
  padding: 6px 12px;
  font-size: 14px;
  cursor: pointer;
  transition: background-color 0.2s;

  &:hover {
    background-color: #b85a09;
  }

  &:active {
    background-color: #9a4a07;
  }
`

const DeciderLogPanel = styled(StyledPanel)`
  width: 640px;
  height: 406px;
  position: fixed;
  top: 718px;
  left: 55px;
  z-index: 1000;
  padding: 15px;
  display: flex;
  flex-direction: column;
`

const LogContainer = styled.div`
  flex: 1;
  overflow-y: auto;
  background-color: #f9f9f9;
  border: 1px solid #ddd;
  border-radius: 4px;
  padding: 10px;
  font-family: 'Courier New', monospace;
  font-size: 13px;
  line-height: 1.4;
  white-space: pre-wrap;
  word-wrap: break-word;
`

const LogEntry = styled.div`
  margin-bottom: 4px;
  color: #333;
`

export const DeciderLogDisplay: React.FC = () => {
  const { deciderLogs, clearDeciderLogs } = useContext(PipelineContext)
  const logContainerRef = useRef<HTMLDivElement>(null)

  // Auto-scroll to bottom when new logs arrive
  useEffect(() => {
    if (logContainerRef.current) {
      logContainerRef.current.scrollTop = logContainerRef.current.scrollHeight
    }
  }, [deciderLogs])

  return (
    <>
      <DeciderLogPanelTitle>
        <span>Decider Logs</span>
        <ClearButton onClick={clearDeciderLogs}>Clear</ClearButton>
      </DeciderLogPanelTitle>
      <DeciderLogPanel>
        <LogContainer ref={logContainerRef}>
          {deciderLogs.length === 0 ? (
            <LogEntry style={{ color: '#999', fontStyle: 'italic' }}>No logs yet...</LogEntry>
          ) : (
            deciderLogs.map((log, index) => <LogEntry key={index}>{log}</LogEntry>)
          )}
        </LogContainer>
      </DeciderLogPanel>
    </>
  )
}

