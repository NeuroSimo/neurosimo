import React from 'react'

import { DataSourceDisplay } from 'components/DataSourceDisplay'
import { ProjectSelector } from 'components/ProjectSelector'
import { SubjectPanel } from 'components/SubjectPanel'
import { ExperimentPanel } from 'components/pipeline/ExperimentConfigPanel'
import { PipelineDiagram } from 'components/pipeline/PipelineDiagram'
import { PipelineLayout } from 'components/pipeline/PipelineLayout'
import { SessionPanel } from 'components/SessionPanel'

export const PipelineView = () => {
  return (
    <PipelineLayout
      setupPrimary={
        <>
          <ProjectSelector />
          <SubjectPanel />
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
