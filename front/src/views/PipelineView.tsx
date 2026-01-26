import React from 'react'

import { DataSourceDisplay } from 'components/DataSourceDisplay'
import { ExperimentPanel } from 'components/pipeline/ExperimentConfigPanel'
import { PipelineDiagram } from 'components/pipeline/PipelineDiagram'
import { PipelineLayout } from 'components/pipeline/PipelineLayout'
import { SessionPanel } from 'components/SessionPanel'

export const PipelineView = () => {
  return (
    <PipelineLayout
      setupPrimary={
        <>
          <ExperimentPanel />
          <DataSourceDisplay />
          <SessionPanel />
        </>
      }
      pipeline={
        <>
          <PipelineDiagram />
        </>
      }
      setupTitle="Configuration"
      pipelineTitle="Pipeline"
    />
  )
}
