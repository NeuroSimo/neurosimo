import React from 'react'
import styled from 'styled-components'

import { StyledPanel, SmallerTitle, Select, palette } from 'styles/General'
import { ToggleSwitch } from 'components/ToggleSwitch'
import { FolderTerminalButtons } from 'components/FolderTerminalButtons'
import { useSession, SessionStateValue } from 'providers/SessionProvider'
import { useSessionConfig } from 'providers/SessionConfigProvider'
import { useDefaultToFirstOption } from 'utils/useDefaultToFirstOption'

/* Distance from the node's outer left edge to its title text (2px status border + padding). */
export const PIPELINE_NODE_TITLE_INSET = 23

const Container = styled(StyledPanel)<{ $enabled: boolean }>`
  width: 505px;
  height: 48px;
  padding: 0 0 0 ${PIPELINE_NODE_TITLE_INSET - 2}px;
  display: flex;
  align-items: center;
  background-color: ${palette.surface};
  border: 1px solid ${palette.border};
  border-left: 2px solid ${props => props.$enabled ? palette.green : palette.borderStrong};
  border-radius: 3px;
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
  color: ${props => props.$enabled ? palette.text : palette.textMuted};
`

const PIPELINE_CONTROL_HEIGHT = 26

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
  height: 18px;
  padding: 0 8px;
  border: 1px solid ${palette.borderStrong};
  border-radius: 3px;
  background-color: transparent;
  color: ${palette.textMuted};
  font-size: 10px;
  font-weight: 600;
  letter-spacing: 0.08em;
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

  const { isDraftLoaded } = useSessionConfig()

  /* Only while the module is in use: a disabled node has no select and keeps its stored
     module, so that toggling it back off and on does not lose the choice. */
  const hasModule = useDefaultToFirstOption(module, modules, onModuleChange, isDraftLoaded && enabled && !isSessionRunning)

  const handleModuleChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    onModuleChange(event.target.value)
  }

  return (
    <Container $enabled={enabled}>
      <HorizontalRow>
        <Title $enabled={enabled}>{title}:</Title>
        <ToggleSwitch type='flat' checked={enabled} onChange={onToggle} disabled={isSessionRunning} />
        {enabled ? (
          <PipelineSelect onChange={handleModuleChange} value={hasModule ? module : ''} disabled={isSessionRunning}>
            {!hasModule && (
              <option value="" disabled>
                —
              </option>
            )}
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
