import React from 'react'
import styled from 'styled-components'
import { CONFIG_PANEL_WIDTH, palette } from 'styles/General'

type Props = {
  setupPrimary: React.ReactNode
  pipeline: React.ReactNode
}

/* Sidebar width: config panel content plus its horizontal padding. */
const SIDEBAR_WIDTH = CONFIG_PANEL_WIDTH + 36

const Layout = styled.div`
  display: grid;
  grid-template-columns: ${SIDEBAR_WIDTH}px 1fr;
  grid-template-rows: 100%;
  gap: 0;
  align-items: stretch;
  max-width: 1600px;
  height: 100%;
  margin: 0;
  width: 100%;
  padding: 0;
`

const SetupPrimary = styled.div`
  display: flex;
  flex-direction: column;
  gap: 0;
  grid-column: 1;
  grid-row: 1;
  background-color: ${palette.surface};
  border-right: 1px solid ${palette.border};
  overflow: hidden;

  /* The menu bar's bottom border already separates the first section from the shell. */
  & > :first-child {
    border-top: none;
  }
`

const PipelineArea = styled.div`
  display: flex;
  flex-direction: column;
  gap: 16px;
  grid-column: 2;
  grid-row: 1;
  align-items: flex-start;
  padding-left: 48px;
  padding-top: 16px;
`

export const PipelineLayout: React.FC<Props> = ({ setupPrimary, pipeline }) => {
  return (
    <Layout>
      <SetupPrimary>
        {setupPrimary}
      </SetupPrimary>
      <PipelineArea>
        {pipeline}
      </PipelineArea>
    </Layout>
  )
}

