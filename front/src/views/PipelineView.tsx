import React from 'react'
import styled from 'styled-components'

import { TabBar } from 'styles/General'

import { EegSimulatorDisplay } from 'components/EegSimulatorDisplay'
import { PipelineLogDisplay } from 'components/PipelineLogDisplay'
import { ProjectSelector } from 'components/ProjectSelector'
import { PreprocessorNode } from 'components/pipeline/PreprocessorNode'
import { DeciderNode } from 'components/pipeline/DeciderNode'
import { PresenterNode } from 'components/pipeline/PresenterNode'
import { TmsNode } from 'components/pipeline/TmsNode'
import { ExperimentPanel } from 'components/pipeline/ExperimentPanel'
import { PipelineConnections } from 'components/pipeline/PipelineConnections'

import { StyledPanel } from 'styles/General'

const InputRow = styled.div`
  display: flex;
  justify-content: flex-start;
  align-items: center;
  gap: 5px;
  margin-bottom: 16px;
`

/* Pipeline definition */
const PipelinePanel = styled.div`
  display: grid;
  grid-template-rows: repeat(2, 1fr);
  grid-template-columns: repeat(4, 1fr);
  width: 370px;
  height: 277px;
  gap: 30px;
  position: relative;
  margin-left: 18px;
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

export const PipelineView = () => {
  return (
    <>
      <ProjectSelector />

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
      <ExperimentPanel />
      <EegSimulatorDisplay />
      <PipelineLogDisplay />
    </>
  )
}
