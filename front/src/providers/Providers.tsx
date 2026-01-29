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
          <PipelineProvider>
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
          </PipelineProvider>
        </ProjectProvider>
      </ParameterProvider>
    </ThemeProvider>
  )
}

export default Providers
