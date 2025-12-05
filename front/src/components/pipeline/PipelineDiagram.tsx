import React from 'react'
import styled from 'styled-components'

import { PreprocessorNode } from 'components/pipeline/PreprocessorNode'
import { DeciderNode } from 'components/pipeline/DeciderNode'
import { PresenterNode } from 'components/pipeline/PresenterNode'
import { TmsNode } from 'components/pipeline/TmsNode'
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
  grid-template-rows: repeat(2, 1fr);
  grid-template-columns: repeat(4, 1fr);
  width: 650px;
  height: 360px;
  gap: 30px;
  position: relative;
  margin: 12px auto 0 auto;
`

const PreprocessorSlot = styled.div`
  grid-row: 1 / 2;
  grid-column: 2 / 3;
`

const DeciderSlot = styled.div`
  grid-row: 1 / 2;
  grid-column: 3 / 4;
`

const PresenterSlot = styled.div`
  grid-row: 1 / 2;
  grid-column: 4 / 5;
`

const TmsSlot = styled.div`
  grid-row: 2 / 3;
  grid-column: 4 / 5;
`

const EegCircle = styled.div`
  display: flex;
  justify-content: center;
  align-items: center;
  grid-row: 1 / 2;
  grid-column: 1 / 2;
  width: 44px;
  height: 44px;
  margin-top: 46px;
  background-color: #d46c0b;
  border-radius: 50%;
  font-weight: bold;
`

export const PipelineDiagram: React.FC = () => (
  <PipelineContainer>
    <PipelinePanel>
      <PipelineConnections />
      <EegCircle>EEG</EegCircle>
      <PreprocessorSlot>
        <PreprocessorNode />
      </PreprocessorSlot>
      <DeciderSlot>
        <DeciderNode />
      </DeciderSlot>
      <PresenterSlot>
        <PresenterNode />
      </PresenterSlot>
      <TmsSlot>
        <TmsNode />
      </TmsSlot>
    </PipelinePanel>
  </PipelineContainer>
)

