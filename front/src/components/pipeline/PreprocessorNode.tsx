import React, { useContext } from 'react'

import { setParameterRos } from 'ros/parameters'
import { PipelineContext } from 'providers/PipelineProvider'
import { PipelineNode } from './PipelineNode'

export const PreprocessorNode: React.FC = () => {
  const { preprocessorEnabled, preprocessorModule, preprocessorList } = useContext(PipelineContext)

  const handleToggle = (next: boolean) => {
    setParameterRos('preprocessor.enabled', next, () => {
      console.log('Preprocessor ' + (next ? 'enabled' : 'disabled'))
    })
  }

  const handleModuleChange = (nextModule: string) => {
    setParameterRos('preprocessor.module', nextModule, () => {
      console.log('Preprocessor set to ' + nextModule)
    })
  }

  return (
    <PipelineNode
      title="Preprocessor"
      enabled={preprocessorEnabled}
      module={preprocessorModule}
      modules={preprocessorList}
      onToggle={handleToggle}
      onModuleChange={handleModuleChange}
    />
  )
}

