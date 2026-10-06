import React from 'react'
import styled from 'styled-components'
import {
  CONFIG_PANEL_WIDTH,
  PIPELINE_LOG_WIDTH,
  palette,
} from 'styles/General'

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

/* The middle workspace runs from the sidebar to the fixed right-hand column and from the menu bar
   to the bottom of the window; its height is independent of the telemetry and logs. The pipeline
   is centred horizontally. Vertically, the free space is split 1:3 above and below it, which puts
   its centre at roughly 31% of the workspace height (at 1440p). */
const PipelineArea = styled.div`
  display: flex;
  flex-direction: column;
  align-items: center;
  grid-column: 2;
  grid-row: 1;
  width: calc(100vw - ${SIDEBAR_WIDTH + PIPELINE_LOG_WIDTH}px);
  box-sizing: border-box;

  &::before,
  &::after {
    content: '';
  }

  &::before {
    flex: 1;
  }

  &::after {
    flex: 3;
  }
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

