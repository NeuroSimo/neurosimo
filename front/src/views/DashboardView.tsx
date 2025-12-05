import React from 'react'

import { EegStreamDisplay } from 'components/EegStreamDisplay'
import { StatisticsDisplay } from 'components/StatisticsDisplay'
import { LatencyDisplay } from 'components/LatencyDisplay'
import { SessionDisplay } from 'components/SessionDisplay'
import { ExperimentStatePanel } from 'components/pipeline/ExperimentStatePanel'
import { PipelineLogDisplay } from 'components/PipelineLogDisplay'

export const DashboardView = () => {
  return (
    <>
      <SessionDisplay />
      <EegStreamDisplay />
      <StatisticsDisplay />
      <LatencyDisplay />
      <ExperimentStatePanel />
      <PipelineLogDisplay />
    </>
  )
}

