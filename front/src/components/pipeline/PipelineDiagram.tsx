import React from 'react'
import styled from 'styled-components'

import { PreprocessorNode } from 'components/pipeline/PreprocessorNode'
import { DeciderNode } from 'components/pipeline/DeciderNode'
import { PresenterNode } from 'components/pipeline/PresenterNode'
import { PipelineConnection } from 'components/pipeline/PipelineConnections'
import { PIPELINE_NODE_TITLE_INSET } from 'components/pipeline/PipelineNode'
import { palette } from 'styles/General'

/* The stages form one left-aligned column joined by connectors on a shared axis. The axis passes
   through the EEG source's center and the start of every node title, and the EEG source's left
   edge lines up with the node edges. */
const CONNECTOR_AXIS_X = PIPELINE_NODE_TITLE_INSET
const EEG_SOURCE_SIZE = CONNECTOR_AXIS_X * 2
const CONNECTOR_LENGTH = 44

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
  font-size: 11px;
  font-weight: 700;
  letter-spacing: 0.04em;
  cursor: move;
  z-index: 10;
`

interface FloatingTitleProps {
  xOffset: number;
  yOffset: number;
}

const FloatingTitle = styled.div<FloatingTitleProps>`
  position: absolute;
  font-size: 11px;
  font-weight: 500;
  color: ${palette.textMuted};
  pointer-events: none;
  z-index: 5;
  left: ${props => props.xOffset}px;
  top: ${props => props.yOffset}px;
`

interface PipelineDiagramProps {
  enabledTitleX?: number;
  enabledTitleY?: number;
  moduleTitleX?: number;
  moduleTitleY?: number;
}

/* Column headings sit just above the Preprocessor node, over its toggle and module select. */
const HEADING_OFFSET_Y = EEG_SOURCE_SIZE + CONNECTOR_LENGTH - 21

export const PipelineDiagram: React.FC<PipelineDiagramProps> = ({
  enabledTitleX = 145,
  enabledTitleY = HEADING_OFFSET_Y,
  moduleTitleX = 252,
  moduleTitleY = HEADING_OFFSET_Y,
}) => (
  <PipelinePanel>
    <EegCircle>EEG</EegCircle>
    <FloatingTitle xOffset={enabledTitleX} yOffset={enabledTitleY}>
      Enabled
    </FloatingTitle>
    <FloatingTitle xOffset={moduleTitleX} yOffset={moduleTitleY}>
      Module
    </FloatingTitle>
    <PipelineConnection axisX={CONNECTOR_AXIS_X} length={CONNECTOR_LENGTH} />
    <PreprocessorNode />
    <PipelineConnection axisX={CONNECTOR_AXIS_X} length={CONNECTOR_LENGTH} />
    <DeciderNode />
    <PipelineConnection axisX={CONNECTOR_AXIS_X} length={CONNECTOR_LENGTH} />
    <PresenterNode />
  </PipelinePanel>
)

