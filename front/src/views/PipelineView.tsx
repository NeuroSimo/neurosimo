import React from 'react'

import { EegSimulatorDisplay } from 'components/EegSimulatorDisplay'
import { PipelineLogDisplay } from 'components/PipelineLogDisplay'
import { ProjectSelector } from 'components/ProjectSelector'
import { ExperimentPanel } from 'components/pipeline/ExperimentPanel'
import { PipelineDiagram } from 'components/pipeline/PipelineDiagram'
import { PipelineLayout } from 'components/pipeline/PipelineLayout'

export const PipelineView = () => {
  return (
    <PipelineLayout>
      <ProjectSelector />
      <PipelineDiagram />
      <ExperimentPanel />
      <EegSimulatorDisplay />
      <PipelineLogDisplay />
    </PipelineLayout>
  )
}
