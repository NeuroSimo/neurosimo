import React, { useContext } from 'react'

import { useSessionConfig } from 'providers/SessionConfigProvider'
import { PipelineConfigContext } from 'providers/PipelineConfigProvider'
import { PipelineNode } from './PipelineNode'

export const DeciderNode: React.FC = () => {
  const { deciderEnabled, deciderModule, deciderList } = useContext(PipelineConfigContext)
  const { setDeciderEnabled, setDeciderModule } = useSessionConfig()

  const handleToggle = (next: boolean) => {
    setDeciderEnabled(next, () => {
      console.log('Decider ' + (next ? 'enabled' : 'disabled'))
    })
  }

  const handleModuleChange = (nextModule: string) => {
    setDeciderModule(nextModule, () => {
      console.log('Decider set to ' + nextModule)
    })
  }

  return (
    <PipelineNode
      title="Decider"
      enabled={deciderEnabled}
      module={deciderModule}
      modules={deciderList}
      onToggle={handleToggle}
      onModuleChange={handleModuleChange}
      folderName="decider"
    />
  )
}

