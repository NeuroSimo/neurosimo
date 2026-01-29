import React from 'react'
import { ThemeProvider } from 'styled-components'
import theme from 'styles/theme'

import { ParameterProvider } from './ParameterProvider'
import { ProjectProvider } from './ProjectProvider'
import { PipelineConfigProvider } from './PipelineConfigProvider'
import { LogProvider } from './LogProvider'
import { ExperimentProvider } from './ExperimentProvider'
import { EegStreamProvider } from './EegStreamProvider'
import { StatisticsProvider } from './StatisticsProvider'
import { EegSimulatorProvider } from './EegSimulatorProvider'
import { EegBridgeProvider } from './EegBridgeProvider'
import { HealthProvider } from './HealthProvider'
import { DiskStatusProvider } from './DiskStatusProvider'
import { SessionProvider } from './SessionProvider'
import { PlaybackProvider } from './PlaybackProvider'

interface Props {
  children: React.ReactNode
}

const Providers: React.FC<Props> = ({ children }) => {
  return (
    <ThemeProvider theme={theme}>
      <ParameterProvider>
        <ProjectProvider>
          <PipelineConfigProvider>
            <LogProvider>
              <ExperimentProvider>
                <EegStreamProvider>
              <StatisticsProvider>
                <EegSimulatorProvider>
                  <EegBridgeProvider>
                    <SessionProvider>
                      <HealthProvider>
                        <DiskStatusProvider>
                          <PlaybackProvider>{children}</PlaybackProvider>
                        </DiskStatusProvider>
                      </HealthProvider>
                    </SessionProvider>
                  </EegBridgeProvider>
                </EegSimulatorProvider>
              </StatisticsProvider>
                </EegStreamProvider>
              </ExperimentProvider>
            </LogProvider>
          </PipelineConfigProvider>
        </ProjectProvider>
      </ParameterProvider>
    </ThemeProvider>
  )
}

export default Providers
