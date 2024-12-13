import React, { useState } from 'react'
import { Route, Routes } from 'react-router-dom'
import styled from 'styled-components'

import './App.css'

import Providers from './providers/Providers'
import { DatasetProvider } from './providers/DatasetProvider'
import { HealthcheckProvider } from './providers/HealthcheckProvider'
import { PipelineProvider } from './providers/PipelineProvider'
import { EegProvider } from './providers/EegProvider'
import { ConfigProvider } from './providers/ConfigProvider'
import { SystemProvider } from './providers/SystemProvider'

import { HealthcheckMessageDisplay } from 'components/HealthcheckMessageDisplay'
import { HealthcheckStatusDisplay } from 'components/HealthcheckStatusDisplay'
import { MultipleViews } from 'views/MultipleViews'
import { Header as StyledHeader } from 'styles/StyledTypography'
import { ProjectProvider } from 'providers/ProjectProvider'

const App = () => {
  return (
    <Providers>
      <ProjectProvider>
        <SystemProvider>
          <PipelineProvider>
            <ConfigProvider>
              <EegProvider>
                <DatasetProvider>
                  <HealthcheckProvider>
                    <Header>mTMS control panel</Header>
                    <HealthcheckStatusDisplay />
                    <HealthcheckMessageDisplay />
                    <Wrapper>
                      <MultipleViews />
                    </Wrapper>
                  </HealthcheckProvider>
                </DatasetProvider>
              </EegProvider>
            </ConfigProvider>
          </PipelineProvider>
        </SystemProvider>
      </ProjectProvider>
    </Providers>
  )
}

const Header = styled(StyledHeader)`
  font-size: 1.8rem;
  color: #333;
  margin-bottom: 1rem;
  padding: 0.5rem;
  background-color: #f2f2f2;
  border-bottom: 2px solid #ddd;
  border-radius: 3px 3px 0 0;
`

const Wrapper = styled.div`
  padding: 1rem;
  margin: 0.5rem;
  background-color: #e8e8e8;
  border-radius: 5px;
  box-shadow: 1px 1px 5px rgba(0, 0, 0, 0.1);
`

export default App
