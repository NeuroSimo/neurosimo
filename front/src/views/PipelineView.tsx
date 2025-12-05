import React, { useContext, useEffect, useState, useRef } from 'react'
import styled from 'styled-components'

import { TabBar } from 'styles/General'

import { EegSimulatorDisplay } from 'components/EegSimulatorDisplay'
import { PipelineLogDisplay } from 'components/PipelineLogDisplay'
import { ProjectSelector } from 'components/ProjectSelector'
import { PreprocessorNode } from 'components/pipeline/PreprocessorNode'
import { DeciderNode } from 'components/pipeline/DeciderNode'
import { PresenterNode } from 'components/pipeline/PresenterNode'
import { TmsNode } from 'components/pipeline/TmsNode'
import { ExperimentPanel } from 'components/pipeline/ExperimentPanel'

import { StyledPanel, ProjectRow, ConfigRow, ConfigLabel, Select, SmallerTitle } from 'styles/General'

import { ToggleSwitch } from 'components/ToggleSwitch'

import {
  setPreprocessorModuleRos,
  setPreprocessorEnabledRos,
  setDeciderModuleRos,
  setDeciderEnabledRos,
  setPresenterModuleRos,
  setPresenterEnabledRos,
  setExperimentProtocolRos,
} from 'ros/pipeline'

import { listProjects, setActiveProject } from 'ros/project'

import { PipelineContext } from 'providers/PipelineProvider'
import { ProjectContext } from 'providers/ProjectProvider'

const InputRow = styled.div`
  display: flex;
  justify-content: flex-start;
  align-items: center;
  gap: 5px;
  margin-bottom: 16px;
`

/* Pipeline definition */
const PipelinePanel = styled.div`
  display: grid;
  grid-template-rows: repeat(2, 1fr);
  grid-template-columns: repeat(4, 1fr);
  width: 370px;
  height: 277px;
  gap: 30px;
  position: relative;
  margin-left: 18px;
`

const PreprocessorSlot = styled.div`
  grid-row: 1 / 2;
  grid-column: 2 / 3;
`

const DeciderSlot = styled.div`
  grid-row: 1 / 2;
  grid-column: 3 / 4;
`

const PresenterSlot = styled.div`
  grid-row: 1 / 2;
  grid-column: 4 / 5;
`

const TmsSlot = styled.div`
  grid-row: 2 / 3;
  grid-column: 4 / 5;
`

const EegCircle = styled.div`
  display: flex;
  justify-content: center;
  align-items: center;
  grid-row: 1 / 2;
  grid-column: 1 / 2;
  width: 44px;
  height: 44px;
  margin-top: 46px;
  background-color: #d46c0b;
  border-radius: 50%;
  font-weight: bold;
`

const Arrow = styled.div`
  width: 18px; /* Arrow width */
  height: 5px; /* Arrow thickness */
  background: #888;
  position: absolute;
  transform: translateY(-50%);

  &:after {
    content: '';
    position: absolute;
    top: 50%;
    right: -5px;
    transform: translateY(-50%);
    border-top: 6px solid transparent;
    border-bottom: 6px solid transparent;
    border-left: 6px solid #888;
  }
`

/* Session storage utilities. */

const getData = (): any => {
  const data = sessionStorage.getItem('pipeline')
  return data ? JSON.parse(data) : {}
}

const storeKey = (key: string, value: any) => {
  const currentData = getData()
  currentData[key] = value
  sessionStorage.setItem('pipeline', JSON.stringify(currentData))
}

const getKey = (key: string, defaultValue: any): any => {
  const data = getData()
  return key in data ? data[key] : defaultValue
}

export const PipelineView = () => {
  const { activeProject } = useContext(ProjectContext)

  const {
    preprocessorList,
    preprocessorModule,
    preprocessorEnabled,
    deciderList,
    deciderModule,
    deciderEnabled,
    presenterList,
    presenterModule,
    presenterEnabled,
    protocolList,
    protocolName,
    experimentState,
  } = useContext(PipelineContext)

  const [projects, setProjects] = useState<string[]>([])

  const handleProjectChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const newActiveProject = event.target.value
    setActiveProject(newActiveProject, () => {
      console.log('Active project set to ' + newActiveProject)
    })
  }

  /* Preprocessor */
  const handlePreprocessorEnabled = (enabled: boolean) => {
    setPreprocessorEnabledRos(enabled, () => {
      console.log('Preprocessor ' + (enabled ? 'enabled' : 'disabled'))
    })
  }

  const handlePreprocessorModuleChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const module = event.target.value
    setPreprocessorModuleRos(module, () => {
      console.log('Preprocessor set to ' + module)
    })
  }

  /* Decider */
  const handleDeciderEnabled = (enabled: boolean) => {
    setDeciderEnabledRos(enabled, () => {
      console.log('Decider ' + (enabled ? 'enabled' : 'disabled'))
    })
  }

  const handleDeciderModuleChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const module = event.target.value
    setDeciderModuleRos(module, () => {
      console.log('Decider set to ' + module)
    })
  }

  /* Presenter */
  const handlePresenterEnabled = (enabled: boolean) => {
    setPresenterEnabledRos(enabled, () => {
      console.log('Presenter ' + (enabled ? 'enabled' : 'disabled'))
    })
  }

  const handlePresenterModuleChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const module = event.target.value
    setPresenterModuleRos(module, () => {
      console.log('Presenter set to ' + module)
    })
  }

  /* Experiment coordinator */
  const handleProtocolChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const protocol = event.target.value
    setExperimentProtocolRos(protocol, () => {
      console.log('Protocol set to ' + protocol)
    })
  }

  const formatSeconds = (value?: number) => {
    if (value === undefined || value === null) return '—'
    return `${value.toFixed(1)}s`
  }

  /* Set list of projects. */
  useEffect(() => {
    listProjects((projects) => {
      setProjects(projects)
    })
  }, [])

  /* Update session storage. */
  useEffect(() => {
    storeKey('preprocessorEnabled', preprocessorEnabled)
  }, [preprocessorEnabled])

  useEffect(() => {
    storeKey('deciderEnabled', deciderEnabled)
  }, [deciderEnabled])

  useEffect(() => {
    storeKey('presenterEnabled', presenterEnabled)
  }, [presenterEnabled])

  return (
    <>
      <ProjectSelector projects={projects} activeProject={activeProject} onChange={handleProjectChange} />

      <PipelinePanel>
        <Arrow style={{ left: '47px', top: '68px' }} />
        <Arrow style={{ left: '252px', top: '68px' }} />
        <Arrow style={{ left: '457px', top: '68px' }} />
        <Arrow style={{ left: '453px', top: '146px', width: '27px', transform: 'rotate(45deg)' }} />
        <EegCircle>EEG</EegCircle>
        <PreprocessorSlot>
          <PreprocessorNode
            enabled={preprocessorEnabled}
            module={preprocessorModule}
            modules={preprocessorList}
            onToggle={handlePreprocessorEnabled}
            onModuleChange={handlePreprocessorModuleChange}
          />
        </PreprocessorSlot>
        <DeciderSlot>
          <DeciderNode
            enabled={deciderEnabled}
            module={deciderModule}
            modules={deciderList}
            onToggle={handleDeciderEnabled}
            onModuleChange={handleDeciderModuleChange}
          />
        </DeciderSlot>
        <PresenterSlot>
          <PresenterNode
            enabled={presenterEnabled}
            module={presenterModule}
            modules={presenterList}
            onToggle={handlePresenterEnabled}
            onModuleChange={handlePresenterModuleChange}
          />
        </PresenterSlot>
        <TmsSlot>
          <TmsNode />
        </TmsSlot>
      </PipelinePanel>
      <ExperimentPanel
        protocolName={protocolName}
        protocolList={protocolList}
        experimentState={experimentState}
        onProtocolChange={handleProtocolChange}
        formatSeconds={formatSeconds}
      />
      <EegSimulatorDisplay />
      <PipelineLogDisplay />
    </>
  )
}
