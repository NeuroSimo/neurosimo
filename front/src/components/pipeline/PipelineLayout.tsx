import React from 'react'
import styled from 'styled-components'
import { CONFIG_PANEL_WIDTH } from 'styles/General'

type Props = {
  setupPrimary: React.ReactNode
  pipeline: React.ReactNode
  setupTitle?: string
  pipelineTitle?: string
}

const Layout = styled.div`
  display: grid;
  grid-template-columns: ${CONFIG_PANEL_WIDTH + 40}px 1fr;
  grid-template-rows: auto;
  gap: 16px 24px;
  align-items: start;
  max-width: 1600px;
  margin: 0 auto;
  width: 100%;
`

const SetupPrimary = styled.div`
  display: flex;
  flex-direction: column;
  gap: 12px;
  grid-column: 1;
  grid-row: 1;
`

const ColumnHeader = styled.div`
  font-size: 14px;
  font-weight: bold;
  margin-bottom: 8px;
`

const PipelineArea = styled.div`
  display: flex;
  flex-direction: column;
  gap: 16px;
  grid-column: 2;
  grid-row: 1;
  align-items: flex-start;
  padding-left: 0px;
`

export const PipelineLayout: React.FC<Props> = ({ setupPrimary, pipeline, setupTitle, pipelineTitle }) => {
  return (
    <Layout>
      <SetupPrimary>
        {setupTitle && <ColumnHeader>{setupTitle}</ColumnHeader>}
        {setupPrimary}
      </SetupPrimary>
      <PipelineArea>
        {pipelineTitle && <ColumnHeader>{pipelineTitle}</ColumnHeader>}
        {pipeline}
      </PipelineArea>
    </Layout>
  )
}

