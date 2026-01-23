import React from 'react'
import { ThemeProvider } from 'styled-components'
import theme from 'styles/theme'

import { ParameterProvider } from './ParameterProvider'
import { ProjectProvider } from './ProjectProvider'
import { PipelineProvider } from './PipelineProvider'
import { EegStreamProvider } from './EegStreamProvider'
import { StatisticsProvider } from './StatisticsProvider'
import { EegSimulatorProvider } from './EegSimulatorProvider'
import { EegBridgeProvider } from './EegBridgeProvider'
import { HealthcheckProvider } from './HealthcheckProvider'
import { SessionProvider } from './SessionProvider'

interface Props {
  children: React.ReactNode
}

const Providers: React.FC<Props> = ({ children }) => {
  return (
    <ThemeProvider theme={theme}>
      <ParameterProvider>
        <ProjectProvider>
          <PipelineProvider>
            <EegStreamProvider>
              <StatisticsProvider>
                <EegSimulatorProvider>
                  <EegBridgeProvider>
                    <SessionProvider>
                      <HealthcheckProvider>{children}</HealthcheckProvider>
                    </SessionProvider>
                  </EegBridgeProvider>
                </EegSimulatorProvider>
              </StatisticsProvider>
            </EegStreamProvider>
          </PipelineProvider>
        </ProjectProvider>
      </ParameterProvider>
    </ThemeProvider>
  )
}

export default Providers
