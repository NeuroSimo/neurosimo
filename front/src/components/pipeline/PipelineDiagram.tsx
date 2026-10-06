import React from 'react'
import styled from 'styled-components'

import { PreprocessorNode } from 'components/pipeline/PreprocessorNode'
import { DeciderNode } from 'components/pipeline/DeciderNode'
import { PresenterNode } from 'components/pipeline/PresenterNode'
import { PipelineConnections } from 'components/pipeline/PipelineConnections'
import { palette } from 'styles/General'

const PipelineContainer = styled.div`
  background: transparent;
  padding: 12px 32px 24px 32px;
  display: flex;
  justify-content: center;
`

const PipelinePanel = styled.div`
  display: grid;
  grid-template-rows: repeat(4, 1fr);
  grid-template-columns: repeat(2, 1fr);
  width: 510px;
  height: 400px;
  gap: 30px;
  position: relative;
  margin: 12px auto 0 auto;
`

/* Nodes are vertically centered in their grid rows; PipelineConnections relies on this. */
const NodeSlot = styled.div`
  display: flex;
  align-items: center;
`

const PreprocessorSlot = styled(NodeSlot)`
  grid-row: 2 / 3;
  grid-column: 1 / 3;
`

const DeciderSlot = styled(NodeSlot)`
  grid-row: 3 / 4;
  grid-column: 1 / 3;
`

const PresenterSlot = styled(NodeSlot)`
  grid-row: 4 / 5;
  grid-column: 1 / 2;
`

const EegCircle = styled.div`
  display: flex;
  justify-content: center;
  align-items: center;
  position: absolute;
  top: 19px;
  left: 10%;
  transform: translateX(-50%);
  width: 44px;
  height: 44px;
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

export const PipelineDiagram: React.FC<PipelineDiagramProps> = ({
  enabledTitleX = 145,
  enabledTitleY = 102,
  moduleTitleX = 252,
  moduleTitleY = 102,
}) => (
  <PipelineContainer>
    <PipelinePanel>
      <PipelineConnections />
      <EegCircle>EEG</EegCircle>
      <FloatingTitle xOffset={enabledTitleX} yOffset={enabledTitleY}>
        Enabled
      </FloatingTitle>
      <FloatingTitle xOffset={moduleTitleX} yOffset={moduleTitleY}>
        Module
      </FloatingTitle>
      <PreprocessorSlot>
        <PreprocessorNode />
      </PreprocessorSlot>
      <DeciderSlot>
        <DeciderNode />
      </DeciderSlot>
      <PresenterSlot>
        <PresenterNode />
      </PresenterSlot>
    </PipelinePanel>
  </PipelineContainer>
)

