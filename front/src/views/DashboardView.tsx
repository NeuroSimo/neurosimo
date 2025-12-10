import React from 'react'

import { EegStreamDisplay } from 'components/EegStreamDisplay'
import { StatisticsDisplay } from 'components/StatisticsDisplay'
import { StimulationDisplay } from 'components/StimulationDisplay'
import { ExperimentStatePanel } from 'components/pipeline/ExperimentStatePanel'
import { PipelineLogDisplay } from 'components/PipelineLogDisplay'

export const DashboardView = () => {
  return (
    <>
      <EegStreamDisplay />
      <StatisticsDisplay />
      <StimulationDisplay />
      <ExperimentStatePanel />
      <PipelineLogDisplay />
    </>
  )
}

