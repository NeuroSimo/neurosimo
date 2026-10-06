import React from 'react'

import { StatisticsDisplay } from 'components/StatisticsDisplay'
import { StimulationDisplay } from 'components/StimulationDisplay'
import { ExperimentStatePanel } from 'components/pipeline/ExperimentStatePanel'
import { PipelineLogDisplay } from 'components/PipelineLogDisplay'

export const DashboardView = () => {
  return (
    <>
      <StatisticsDisplay />
      <StimulationDisplay />
      <ExperimentStatePanel />
      <PipelineLogDisplay />
    </>
  )
}

