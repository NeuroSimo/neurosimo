import React, { useContext } from 'react'

import { setParameterRos } from 'ros/parameters'
import { PipelineContext } from 'providers/PipelineProvider'
import { PipelineNode } from './PipelineNode'

export const DeciderNode: React.FC = () => {
  const { deciderEnabled, deciderModule, deciderList } = useContext(PipelineContext)

  const handleToggle = (next: boolean) => {
    setParameterRos('decider.enabled', next, () => {
      console.log('Decider ' + (next ? 'enabled' : 'disabled'))
    })
  }

  const handleModuleChange = (nextModule: string) => {
    setParameterRos('decider.module', nextModule, () => {
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
    />
  )
}

