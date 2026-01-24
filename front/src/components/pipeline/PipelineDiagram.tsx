import React from 'react'
import styled from 'styled-components'

import { PreprocessorNode } from 'components/pipeline/PreprocessorNode'
import { DeciderNode } from 'components/pipeline/DeciderNode'
import { PresenterNode } from 'components/pipeline/PresenterNode'
import { PipelineConnections } from 'components/pipeline/PipelineConnections'

const PipelineContainer = styled.div`
  background: #fafafa;
  border-radius: 8px;
  padding: 24px 32px;
  box-shadow: 0px 2px 6px rgba(0, 0, 0, 0.08);
  display: flex;
  justify-content: center;
`

const PipelinePanel = styled.div`
  display: grid;
  grid-template-rows: repeat(4, 1fr);
  grid-template-columns: repeat(2, 1fr);
  width: 500px;
  height: 400px;
  gap: 30px;
  position: relative;
  margin: 12px auto 0 auto;
`

const PreprocessorSlot = styled.div`
  grid-row: 2 / 3;
  grid-column: 1 / 3;
`

const DeciderSlot = styled.div`
  grid-row: 3 / 4;
  grid-column: 1 / 3;
`

const PresenterSlot = styled.div`
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
  background-color: #d46c0b;
  border-radius: 50%;
  font-weight: bold;
  cursor: move;
  z-index: 10;
`

interface FloatingTitleProps {
  xOffset: number;
  yOffset: number;
}

const FloatingTitle = styled.div<FloatingTitleProps>`
  position: absolute;
  font-size: 12px;
  font-weight: 600;
  color: #666;
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
  enabledTitleY = 85,
  moduleTitleX = 252,
  moduleTitleY = 85,
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

