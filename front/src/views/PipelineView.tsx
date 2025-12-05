import React from 'react'

import { EegSimulatorDisplay } from 'components/EegSimulatorDisplay'
import { ProjectSelector } from 'components/ProjectSelector'
import { ExperimentPanel } from 'components/pipeline/ExperimentPanel'
import { PipelineDiagram } from 'components/pipeline/PipelineDiagram'
import { PipelineLayout } from 'components/pipeline/PipelineLayout'

export const PipelineView = () => {
  return (
    <PipelineLayout
      setupPrimary={
        <>
          <ProjectSelector />
          <ExperimentPanel />
          <EegSimulatorDisplay />
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
