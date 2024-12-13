import React, { useState, useEffect } from 'react'
import styled from 'styled-components'

import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'
import { faCog } from '@fortawesome/free-solid-svg-icons'

import DataVisualize from './DataVisualize'
import DataVisualizeWebGL from './DataVisualizeWebGL'

import Targets from './Targets'
import { SmallHeader } from '../styles/StyledTypography'

import { ConfigView } from './ConfigView'
import { SystemView } from './SystemView'
import { ExperimentView } from './ExperimentView'
import { PipelineView } from './PipelineView'

/* Session storage utilities. */

const storeKey = (key: string, value: any) => {
  sessionStorage.setItem(key, JSON.stringify(value))
}

const getKey = (key: string, defaultValue: any): any => {
  const storedValue = sessionStorage.getItem(key)
  return storedValue !== null ? JSON.parse(storedValue) : defaultValue
}

export const MultipleViews = () => {
  const [currentView, setCurrentView] = useState(() => getKey('currentView', 'SystemView'))

  useEffect(() => {
    storeKey('currentView', currentView)
  }, [currentView])

  return (
    <div>
      <OptionWrapper>
        <a
          href='#'
          onClick={() => setCurrentView('SystemView')}
          className={currentView === 'SystemView' ? 'active' : ''}
        >
          System
        </a>
        <a
          href='#'
          onClick={() => setCurrentView('experiment')}
          className={currentView === 'experiment' ? 'active' : ''}
        >
          Experiment
        </a>
        <a href='#' onClick={() => setCurrentView('pipeline')} className={currentView === 'pipeline' ? 'active' : ''}>
          Pipeline
        </a>
        <ConfigIcon onClick={() => setCurrentView('config')} className={currentView === 'config' ? 'active' : ''}>
          <FontAwesomeIcon icon={faCog} />
        </ConfigIcon>
      </OptionWrapper>
      <ViewContainer>
        <Wrapper style={{ display: currentView === 'SystemView' ? 'block' : 'none' }}>
          <SmallHeader>System</SmallHeader>
          <SystemView />
        </Wrapper>
        <Wrapper style={{ display: currentView === 'experiment' ? 'block' : 'none' }}>
          <SmallHeader>Experiment</SmallHeader>
          <ExperimentView />
        </Wrapper>
        <Wrapper style={{ display: currentView === 'pipeline' ? 'block' : 'none' }}>
          <SmallHeader>Pipeline</SmallHeader>
          <PipelineView />
        </Wrapper>
        <Wrapper style={{ display: currentView === 'config' ? 'block' : 'none' }}>
          <SmallHeader>Config</SmallHeader>
          <ConfigView />
        </Wrapper>
      </ViewContainer>
    </div>
  )
}

const ViewContainer = styled.div`
  display: flex;
  flex-wrap: wrap;
`

const OptionWrapper = styled.div`
  margin: 0.5rem;

  a {
    text-decoration: none;
    color: #505050; // Darker gray for regular links
    padding: 0.5rem;
    display: inline-block;
    transition: color 0.3s ease;

    &:hover {
      color: #303030; // Even darker gray for hover
    }

    &.active {
      color: #222222; // Almost black for active link
      font-weight: bold;
    }
  }
`

const Wrapper = styled.div`
  width: 100%;
  padding: 0.5rem;
  margin: 0.5rem;
  border: 3px solid ${(p) => p.theme.colors.darkgray};
`

const ConfigIcon = styled.a`
  position: absolute;
  top: 105px;
  right: 500px;
  font-size: 28px;
  cursor: pointer;
`
