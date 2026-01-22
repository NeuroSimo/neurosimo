import React from 'react'

import { DataSourceDisplay } from 'components/DataSourceDisplay'
import { EegDeviceDisplay } from 'components/EegDeviceDisplay'
import { ProjectSelector } from 'components/ProjectSelector'
import { ExperimentPanel } from 'components/pipeline/ExperimentConfigPanel'
import { PipelineDiagram } from 'components/pipeline/PipelineDiagram'
import { PipelineLayout } from 'components/pipeline/PipelineLayout'

export const PipelineView = () => {
  return (
    <PipelineLayout
      setupPrimary={
        <>
          <ProjectSelector />
          <ExperimentPanel />
          <DataSourceDisplay />
          <EegDeviceDisplay />
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
