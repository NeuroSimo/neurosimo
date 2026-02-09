import React from 'react'
import { ThemeProvider } from 'styled-components'
import theme from 'styles/theme'

import { SessionConfigProvider } from './SessionConfigProvider'
import { ProjectProvider } from './ProjectProvider'
import { PipelineConfigProvider } from './PipelineConfigProvider'
import { PipelineProvider } from './PipelineProvider'
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
import { ExporterProvider } from './ExporterProvider'

interface Props {
  children: React.ReactNode
}

const Providers: React.FC<Props> = ({ children }) => {
  return (
    <ThemeProvider theme={theme}>
      <SessionConfigProvider>
        <ProjectProvider>
          <PipelineConfigProvider>
            <PipelineProvider>
              <LogProvider>
              <ExperimentProvider>
                <EegStreamProvider>
              <StatisticsProvider>
                <EegSimulatorProvider>
                  <EegBridgeProvider>
                    <SessionProvider>
                      <HealthProvider>
                        <DiskStatusProvider>
                          <ExporterProvider>
                            <PlaybackProvider>{children}</PlaybackProvider>
                          </ExporterProvider>
                        </DiskStatusProvider>
                      </HealthProvider>
                    </SessionProvider>
                  </EegBridgeProvider>
                </EegSimulatorProvider>
              </StatisticsProvider>
                </EegStreamProvider>
                </ExperimentProvider>
              </LogProvider>
            </PipelineProvider>
          </PipelineConfigProvider>
        </ProjectProvider>
      </SessionConfigProvider>
    </ThemeProvider>
  )
}

export default Providers
