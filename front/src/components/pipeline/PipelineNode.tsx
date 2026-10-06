import React from 'react'
import styled from 'styled-components'

import { StyledPanel, SmallerTitle, Select, palette } from 'styles/General'
import { ToggleSwitch } from 'components/ToggleSwitch'
import { FolderTerminalButtons } from 'components/FolderTerminalButtons'
import { useSession, SessionStateValue } from 'providers/SessionProvider'
import { useSessionConfig } from 'providers/SessionConfigProvider'
import { useDefaultToFirstOption } from 'utils/useDefaultToFirstOption'

/* Visual scale of the pipeline cluster relative to its base design. Sizes, padding, fonts and the
   gaps between stages scale with it; the horizontal gaps between node controls and the module
   select width do not, so the node grows in width only by what its scaled contents need. */
export const PIPELINE_SCALE = 1.12
export const scaled = (px: number) => Math.round(px * PIPELINE_SCALE)

/* Distance from the node's outer left edge to its title text (2px status border + padding). */
export const PIPELINE_NODE_TITLE_INSET = scaled(23)

const TITLE_WIDTH = scaled(85)
const TITLE_GAP = 18
/* The toggle is sized in em, so it scales with this font size; its side margins stay fixed. */
const TOGGLE_FONT_SIZE = 13 * PIPELINE_SCALE
const TOGGLE_WIDTH = 2.65 * TOGGLE_FONT_SIZE
const TOGGLE_MARGIN = 26
const MODULE_SELECT_GAP = 40
const MODULE_SELECT_WIDTH = 200

/* Horizontal positions within a node, measured from its outer left edge (for the column headings). */
export const PIPELINE_NODE_TOGGLE_RIGHT =
  PIPELINE_NODE_TITLE_INSET + TITLE_WIDTH + TITLE_GAP + TOGGLE_MARGIN + TOGGLE_WIDTH
export const PIPELINE_NODE_SELECT_LEFT = PIPELINE_NODE_TOGGLE_RIGHT + TOGGLE_MARGIN + MODULE_SELECT_GAP

/* Space between the terminal icon button and the node's right edge. The glyph itself sits ~4px
   inside its button, so its visible inset is about 5px more than this. */
const RIGHT_PADDING = 15

const NODE_HEIGHT = scaled(48)
/* Rendered height including the 1px top and bottom borders. */
export const PIPELINE_NODE_OUTER_HEIGHT = NODE_HEIGHT + 2

const Container = styled(StyledPanel)<{ $enabled: boolean }>`
  width: 510px;
  height: ${NODE_HEIGHT}px;
  padding: 0 ${RIGHT_PADDING}px 0 ${PIPELINE_NODE_TITLE_INSET - 2}px;
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
  width: ${TITLE_WIDTH}px;
  flex-shrink: 0;
  text-align: left;
  margin-bottom: 0;
  margin-top: 0;
  margin-right: ${TITLE_GAP}px;
  font-size: ${12 * PIPELINE_SCALE}px;
  color: ${props => props.$enabled ? palette.text : palette.textMuted};
`

const ToggleScale = styled.div`
  font-size: ${TOGGLE_FONT_SIZE}px;

  .tg-list-item {
    margin: 0 ${TOGGLE_MARGIN}px;
  }
`

const ButtonsScale = styled.div`
  button {
    font-size: ${13 * PIPELINE_SCALE}px;
  }
`

const PIPELINE_CONTROL_HEIGHT = scaled(26)

const PipelineSelect = styled(Select)`
  margin-left: ${MODULE_SELECT_GAP}px;
  width: ${MODULE_SELECT_WIDTH}px;
  min-width: ${MODULE_SELECT_WIDTH}px;
  height: ${PIPELINE_CONTROL_HEIGHT}px;
  font-size: ${12 * PIPELINE_SCALE}px;
  box-sizing: border-box;
  flex-shrink: 0;
`

const DisabledSlot = styled.div`
  margin-left: ${MODULE_SELECT_GAP}px;
  margin-right: 17px;
  width: ${MODULE_SELECT_WIDTH}px;
  min-width: ${MODULE_SELECT_WIDTH}px;
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
  height: ${scaled(18)}px;
  padding: 0 ${scaled(8)}px;
  border: 1px solid ${palette.borderStrong};
  border-radius: 3px;
  background-color: transparent;
  color: ${palette.textMuted};
  font-size: ${10 * PIPELINE_SCALE}px;
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
        <ToggleScale>
          <ToggleSwitch type='flat' checked={enabled} onChange={onToggle} disabled={isSessionRunning} />
        </ToggleScale>
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
        <ButtonsScale>
          <FolderTerminalButtons folderName={folderName} size={scaled(22)} />
        </ButtonsScale>
      </HorizontalRow>
    </Container>
  )
}
