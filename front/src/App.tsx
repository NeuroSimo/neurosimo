import React, { useState } from 'react'
import { Route, Routes } from 'react-router-dom'
import styled from 'styled-components'

import './App.css'

import Providers from './providers/Providers'
import { EegSimulatorProvider } from './providers/EegSimulatorProvider'
import { HealthcheckProvider } from './providers/HealthcheckProvider'
import { PipelineProvider } from './providers/PipelineProvider'
import { EegStreamProvider } from './providers/EegStreamProvider'
import { StatisticsProvider } from './providers/StatisticsProvider'

import { HealthcheckMessageDisplay } from 'components/HealthcheckMessageDisplay'
import { HealthcheckStatusDisplay } from 'components/HealthcheckStatusDisplay'
import { PipelineView } from 'views/PipelineView'
import { DashboardView } from 'views/DashboardView'
import { Header as StyledHeader } from 'styles/StyledTypography'
import { ProjectProvider } from 'providers/ProjectProvider'

const App = () => {
  return (
    <Providers>
      <Header>NeuroSimo</Header>
      <HealthcheckStatusDisplay />
      <HealthcheckMessageDisplay />
      <Wrapper>
        <PipelineView />
        <DashboardView />
      </Wrapper>
    </Providers>
  )
}

const Header = styled(StyledHeader)`
  font-size: 1.02rem;
  color: #333;
  margin-bottom: 0.62rem;
  padding: 0.31rem;
  background-color: #f2f2f2;
  border-bottom: 2px solid #ddd;
  border-radius: 2px 2px 0 0;
`

const Wrapper = styled.div`
  padding: 1.23rem 0rem;
  margin: 0.62rem;
  background-color: #e8e8e8;
  border-radius: 16px;
  box-shadow: 1px 1px 4px rgba(0, 0, 0, 0.1);
`

export default App
