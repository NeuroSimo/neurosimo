import React, { useContext } from 'react'

import { useSessionConfig } from 'providers/SessionConfigProvider'
import { PipelineConfigContext } from 'providers/PipelineConfigProvider'
import { PipelineNode } from './PipelineNode'

export const PresenterNode: React.FC = () => {
  const { presenterEnabled, presenterModule, presenterList } = useContext(PipelineConfigContext)
  const { setPresenterEnabled, setPresenterModule } = useSessionConfig()

  const handleToggle = (next: boolean) => {
    setPresenterEnabled(next, () => {
      console.log('Presenter ' + (next ? 'enabled' : 'disabled'))
    })
  }

  const handleModuleChange = (nextModule: string) => {
    setPresenterModule(nextModule, () => {
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
      folderName="presenter"
    />
  )
}

