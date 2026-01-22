import React, { useContext } from 'react'

import { setParameterRos } from 'ros/parameters'
import { PipelineContext } from 'providers/PipelineProvider'
import { PipelineNode } from './PipelineNode'

export const PresenterNode: React.FC = () => {
  const { presenterEnabled, presenterModule, presenterList } = useContext(PipelineContext)

  const handleToggle = (next: boolean) => {
    setParameterRos('presenter.enabled', next, () => {
      console.log('Presenter ' + (next ? 'enabled' : 'disabled'))
    })
  }

  const handleModuleChange = (nextModule: string) => {
    setParameterRos('presenter.module', nextModule, () => {
      console.log('Presenter set to ' + nextModule)
    })
  }

  return (
    <PipelineNode
      title="Presenter"
      enabled={presenterEnabled}
      module={presenterModule}
      modules={presenterList}
      onToggle={handleToggle}
      onModuleChange={handleModuleChange}
    />
  )
}

