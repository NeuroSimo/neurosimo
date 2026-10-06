import React, { useContext } from 'react'
import styled from 'styled-components'

import { PreprocessorNode } from 'components/pipeline/PreprocessorNode'
import { DeciderNode } from 'components/pipeline/DeciderNode'
import { PresenterNode } from 'components/pipeline/PresenterNode'
import { PipelineBypass, PipelineConnection } from 'components/pipeline/PipelineConnections'
import {
  PIPELINE_NODE_OUTER_HEIGHT,
  PIPELINE_NODE_SELECT_LEFT,
  PIPELINE_NODE_TITLE_INSET,
  PIPELINE_NODE_TOGGLE_RIGHT,
  PIPELINE_SCALE,
  scaled,
} from 'components/pipeline/PipelineNode'
import { ModuleListContext } from 'providers/ModuleListProvider'
import { palette } from 'styles/General'

/* The stages form one left-aligned column joined by connectors on a shared axis. The axis passes
   through the EEG source's center and the start of every node title, and the EEG source's left
   edge lines up with the node edges. */
const CONNECTOR_AXIS_X = PIPELINE_NODE_TITLE_INSET
const EEG_SOURCE_SIZE = CONNECTOR_AXIS_X * 2
const CONNECTOR_LENGTH = scaled(44)

const PipelinePanel = styled.div`
  display: flex;
  flex-direction: column;
  align-items: flex-start;
  position: relative;
`

const EegCircle = styled.div`
  display: flex;
  justify-content: center;
  align-items: center;
  flex-shrink: 0;
  width: ${EEG_SOURCE_SIZE}px;
  height: ${EEG_SOURCE_SIZE}px;
  box-sizing: border-box;
  background-color: rgba(229, 149, 74, 0.14);
  border: 1.5px solid ${palette.orange};
  color: ${palette.orange};
  border-radius: 50%;
  font-size: ${11 * PIPELINE_SCALE}px;
  font-weight: 700;
  letter-spacing: 0.04em;
  cursor: move;
  z-index: 10;
`

interface FloatingTitleProps {
  xOffset: number;
  yOffset: number;
  $alignRight?: boolean;
}

const FloatingTitle = styled.div<FloatingTitleProps>`
  position: absolute;
  font-size: ${11 * PIPELINE_SCALE}px;
  font-weight: 500;
  color: ${palette.textMuted};
  pointer-events: none;
  z-index: 5;
  left: ${props => props.xOffset}px;
  top: ${props => props.yOffset}px;
  ${props => props.$alignRight && 'transform: translateX(-100%);'}
`

interface PipelineDiagramProps {
  enabledTitleX?: number;
  enabledTitleY?: number;
  moduleTitleX?: number;
  moduleTitleY?: number;
}

/* Column headings sit just above the Preprocessor node: "Enabled" ends at the toggle's right edge
   and "Module" starts at the module select's left edge. */
const HEADING_OFFSET_Y = EEG_SOURCE_SIZE + CONNECTOR_LENGTH - scaled(21)

/* With the Preprocessor bypassed, the Decider reads the EEG stream directly; the bypass runs from
   the EEG source's left edge (at its centre) into the Decider node's left edge (at its centre). */
const BYPASS_FROM_Y = EEG_SOURCE_SIZE / 2
const BYPASS_TO_Y = EEG_SOURCE_SIZE + 2 * CONNECTOR_LENGTH + PIPELINE_NODE_OUTER_HEIGHT * 1.5

export const PipelineDiagram: React.FC<PipelineDiagramProps> = ({
  enabledTitleX = PIPELINE_NODE_TOGGLE_RIGHT,
  enabledTitleY = HEADING_OFFSET_Y,
  moduleTitleX = PIPELINE_NODE_SELECT_LEFT,
  moduleTitleY = HEADING_OFFSET_Y,
}) => {
  const { preprocessorEnabled, deciderEnabled, presenterEnabled } = useContext(ModuleListContext)

  /* A connector is active when data flows along it: a disabled Decider processes no samples, and
     a disabled Presenter receives nothing. */
  return (
    <PipelinePanel>
      <EegCircle>EEG</EegCircle>
      <FloatingTitle xOffset={enabledTitleX} yOffset={enabledTitleY} $alignRight>
        Enabled
      </FloatingTitle>
      <FloatingTitle xOffset={moduleTitleX} yOffset={moduleTitleY}>
        Module
      </FloatingTitle>
      {!preprocessorEnabled && (
        <PipelineBypass fromY={BYPASS_FROM_Y} toY={BYPASS_TO_Y} active={deciderEnabled} />
      )}
      <PipelineConnection axisX={CONNECTOR_AXIS_X} length={CONNECTOR_LENGTH} active={preprocessorEnabled} />
      <PreprocessorNode />
      <PipelineConnection
        axisX={CONNECTOR_AXIS_X}
        length={CONNECTOR_LENGTH}
        active={preprocessorEnabled && deciderEnabled}
      />
      <DeciderNode />
      <PipelineConnection
        axisX={CONNECTOR_AXIS_X}
        length={CONNECTOR_LENGTH}
        active={deciderEnabled && presenterEnabled}
      />
      <PresenterNode />
    </PipelinePanel>
  )
}

