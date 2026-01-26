import React, { useContext } from 'react'

import { useParameters } from 'providers/ParameterProvider'
import { PipelineContext } from 'providers/PipelineProvider'
import { PipelineNode } from './PipelineNode'

export const DeciderNode: React.FC = () => {
  const { deciderEnabled, deciderModule, deciderList } = useContext(PipelineContext)
  const { setDeciderEnabled, setDeciderModule } = useParameters()

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

