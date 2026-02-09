import React, { useState } from 'react'
import { Route, Routes } from 'react-router-dom'
import styled from 'styled-components'

import './App.css'

import Providers from './providers/Providers'

import { HealthcheckMessageDisplay } from 'components/HealthcheckMessageDisplay'
import { HealthDisplay } from 'components/HealthDisplay'
import { PipelineView } from 'views/PipelineView'
import { DashboardView } from 'views/DashboardView'
import { ProjectProvider } from 'providers/ProjectProvider'
import { DetachedExperimentView } from 'components/DetachedExperimentView'

const App = () => {
  const isDetached = new URLSearchParams(window.location.search).get('detached') === 'true'

  if (isDetached) {
    return (
      <Providers>
        <DetachedExperimentView />
      </Providers>
    )
  }

  return (
    <Providers>
      <HealthDisplay />
      <HealthcheckMessageDisplay />
      <Wrapper>
        <PipelineView />
        <DashboardView />
      </Wrapper>
    </Providers>
  )
}

const Wrapper = styled.div`
  padding: 1.23rem 0rem;
  margin: 0.62rem;
  background-color: #e8e8e8;
  border-radius: 16px;
  box-shadow: 1px 1px 4px rgba(0, 0, 0, 0.1);
`

export default App
