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

const PipelineSelect = styled(Select)`
  margin-left: 40px;
  width: 200px;
  min-width: 200px;
  flex-shrink: 0;
`

interface PipelineNodeProps {
  title: string
  enabled: boolean
  module: string
  modules: string[]
  onToggle: (enabled: boolean) => void
  onModuleChange: (module: string) => void
  folderName: string
}

export const PipelineNode: React.FC<PipelineNodeProps> = ({
  title,
  enabled,
  module,
  modules,
  onToggle,
  onModuleChange,
  folderName,
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
        <PipelineSelect onChange={handleModuleChange} value={module} disabled={isSessionRunning}>
          {modules.map((mod, index) => (
            <option key={index} value={mod}>
              {mod}
            </option>
          ))}
        </PipelineSelect>
        <FolderTerminalButtons folderName={folderName} />
      </HorizontalRow>
    </Container>
  )
}
