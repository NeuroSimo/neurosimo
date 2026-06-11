import React from 'react'
import styled from 'styled-components'

import { StyledPanel, SmallerTitle, Select } from 'styles/General'
import { ToggleSwitch } from 'components/ToggleSwitch'
import { FolderTerminalButtons } from 'components/FolderTerminalButtons'
import { useSession, SessionStateValue } from 'providers/SessionProvider'

const Container = styled(StyledPanel)`
  width: 505px;
  height: 20px;
  background-color: #e6ebf2;
  box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.8), 0 3px 10px rgba(0, 0, 0, 0.14);
`

const HorizontalRow = styled.div`
  display: flex;
  align-items: center;
  gap: 0px;
`

const Title = styled(SmallerTitle)<{ $enabled: boolean }>`
  width: 85px;
  flex-shrink: 0;
  text-align: left;
  margin-bottom: 0;
  margin-top: 0;
  color: ${props => props.$enabled ? 'inherit' : '#999'};
`

const PIPELINE_CONTROL_HEIGHT = 31

const PipelineSelect = styled(Select)`
  margin-left: 40px;
  width: 200px;
  min-width: 200px;
  height: ${PIPELINE_CONTROL_HEIGHT}px;
  box-sizing: border-box;
  flex-shrink: 0;
`

const DisabledSlot = styled.div`
  margin-left: 40px;
  margin-right: 17px;
  width: 200px;
  min-width: 200px;
  height: ${PIPELINE_CONTROL_HEIGHT}px;
  flex-shrink: 0;
  box-sizing: border-box;
  display: flex;
  align-items: center;
  justify-content: flex-start;
`

const DisabledPill = styled.span`
  display: inline-flex;
  align-items: center;
  padding: 3px 10px;
  border: 1px solid #a0a0a0;
  border-radius: 11px;
  background-color: #e4e4e4;
  color: #4d4d4d;
  font-size: 11px;
  font-weight: 600;
  letter-spacing: 0.5px;
  text-transform: uppercase;
`

interface PipelineNodeProps {
  title: string
  enabled: boolean
  module: string
  modules: string[]
  onToggle: (enabled: boolean) => void
  onModuleChange: (module: string) => void
  folderName: string
  disabledLabel?: string
}

export const PipelineNode: React.FC<PipelineNodeProps> = ({
  title,
  enabled,
  module,
  modules,
  onToggle,
  onModuleChange,
  folderName,
  disabledLabel,
}) => {
  const { sessionState } = useSession()
  const isSessionRunning = sessionState.state === SessionStateValue.RUNNING

  const handleModuleChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    onModuleChange(event.target.value)
  }

  return (
    <Container>
      <HorizontalRow>
        <Title $enabled={enabled}>{title}:</Title>
        <ToggleSwitch type='flat' checked={enabled} onChange={onToggle} disabled={isSessionRunning} />
        {enabled ? (
          <PipelineSelect onChange={handleModuleChange} value={module} disabled={isSessionRunning}>
            {modules.map((mod, index) => (
              <option key={index} value={mod}>
                {mod}
              </option>
            ))}
          </PipelineSelect>
        ) : (
          <DisabledSlot>
            <DisabledPill>{disabledLabel}</DisabledPill>
          </DisabledSlot>
        )}
        <FolderTerminalButtons folderName={folderName} />
      </HorizontalRow>
    </Container>
  )
}
