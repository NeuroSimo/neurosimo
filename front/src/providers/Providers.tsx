import React from 'react'
import { ThemeProvider } from 'styled-components'
import theme from 'styles/theme'

import { SessionConfigProvider } from './SessionConfigProvider'
import { ProjectProvider } from './ProjectProvider'
import { ModuleListProvider } from './ModuleListProvider'
import { PipelineProvider } from './PipelineProvider'
import { LogProvider } from './LogProvider'
import { ExperimentProvider } from './ExperimentProvider'
import { EegStreamProvider } from './EegStreamProvider'
import { EegStatisticsProvider } from './EegStatisticsProvider'
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
          <ModuleListProvider>
            <PipelineProvider>
              <LogProvider>
              <ExperimentProvider>
                <EegStreamProvider>
              <EegStatisticsProvider>
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
              </EegStatisticsProvider>
                </EegStreamProvider>
                </ExperimentProvider>
              </LogProvider>
            </PipelineProvider>
          </ModuleListProvider>
        </ProjectProvider>
      </SessionConfigProvider>
    </ThemeProvider>
  )
}

export default Providers
