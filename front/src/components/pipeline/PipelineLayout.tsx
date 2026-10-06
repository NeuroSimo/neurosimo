import React from 'react'
import styled from 'styled-components'
import {
  CONFIG_PANEL_WIDTH,
  DASHBOARD_PANEL_OFFSET_FROM_TOP,
  PIPELINE_LOG_OFFSET_FROM_TOP,
  PIPELINE_LOG_WIDTH,
  palette,
} from 'styles/General'
import { STATUS_STRIP_TOP } from 'components/HealthcheckMessageDisplay'

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

/* The visible middle workspace runs from the sidebar to the fixed right-hand column. The pipeline
   is centred in it horizontally, and vertically within the band of the telemetry panels (from
   their headings down to the top of the logs), so it follows those if they are resized. */
const PipelineArea = styled.div`
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  grid-column: 2;
  grid-row: 1;
  width: calc(100vw - ${SIDEBAR_WIDTH + PIPELINE_LOG_WIDTH}px);
  height: ${PIPELINE_LOG_OFFSET_FROM_TOP - STATUS_STRIP_TOP}px;
  padding-top: ${DASHBOARD_PANEL_OFFSET_FROM_TOP - STATUS_STRIP_TOP}px;
  box-sizing: border-box;
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

