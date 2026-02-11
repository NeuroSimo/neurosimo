import React from 'react'
import { ThemeProvider } from 'styled-components'
import theme from 'styles/theme'

import { SessionConfigProvider } from './SessionConfigProvider'
import { GlobalConfigProvider } from './GlobalConfigProvider'
import { ModuleListProvider } from './ModuleListProvider'
import { SessionStatisticsProvider } from './SessionStatisticsProvider'
import { LogProvider } from './LogProvider'
import { ExperimentProvider } from './ExperimentProvider'
import { EegStreamProvider } from './EegStreamProvider'
import { EegStatisticsProvider } from './EegStatisticsProvider'
import { EegSimulatorProvider } from './EegSimulatorProvider'
import { EegBridgeProvider } from './EegBridgeProvider'
import { HealthProvider } from './HealthProvider'
import { DiskStatusProvider } from './DiskStatusProvider'
import { SessionProvider } from './SessionProvider'
import { RecordingProvider } from './RecordingProvider'
import { ExporterProvider } from './ExporterProvider'

interface Props {
  children: React.ReactNode
}

const Providers: React.FC<Props> = ({ children }) => {
  return (
    <ThemeProvider theme={theme}>
      <SessionConfigProvider>
        <GlobalConfigProvider>
          <ModuleListProvider>
            <SessionStatisticsProvider>
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
                            <RecordingProvider>{children}</RecordingProvider>
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
            </SessionStatisticsProvider>
          </ModuleListProvider>
        </GlobalConfigProvider>
      </SessionConfigProvider>
    </ThemeProvider>
  )
}

export default Providers
