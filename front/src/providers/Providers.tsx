import React from 'react'
import { ThemeProvider } from 'styled-components'
import theme from 'styles/theme'

import { ProjectProvider } from './ProjectProvider'
import { PipelineProvider } from './PipelineProvider'
import { EegStreamProvider } from './EegStreamProvider'
import { StatisticsProvider } from './StatisticsProvider'
import { EegSimulatorProvider } from './EegSimulatorProvider'
import { HealthcheckProvider } from './HealthcheckProvider'

interface Props {
  children: React.ReactNode
}

const Providers: React.FC<Props> = ({ children }) => {
  return (
    <ThemeProvider theme={theme}>
      <ProjectProvider>
        <PipelineProvider>
          <EegStreamProvider>
            <StatisticsProvider>
              <EegSimulatorProvider>
                <HealthcheckProvider>{children}</HealthcheckProvider>
              </EegSimulatorProvider>
            </StatisticsProvider>
          </EegStreamProvider>
        </PipelineProvider>
      </ProjectProvider>
    </ThemeProvider>
  )
}

export default Providers
