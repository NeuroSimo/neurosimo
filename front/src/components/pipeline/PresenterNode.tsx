import React, { useContext } from 'react'

import { setPresenterEnabledRos, setPresenterModuleRos } from 'ros/pipeline'
import { PipelineContext } from 'providers/PipelineProvider'
import { PipelineNode } from './PipelineNode'

export const PresenterNode: React.FC = () => {
  const { presenterEnabled, presenterModule, presenterList } = useContext(PipelineContext)

  const handleToggle = (next: boolean) => {
    setPresenterEnabledRos(next, () => {
      console.log('Presenter ' + (next ? 'enabled' : 'disabled'))
    })
  }

  const handleModuleChange = (nextModule: string) => {
    setPresenterModuleRos(nextModule, () => {
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

