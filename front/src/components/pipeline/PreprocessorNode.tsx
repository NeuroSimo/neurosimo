import React, { useContext } from 'react'

import { setPreprocessorEnabledRos, setPreprocessorModuleRos } from 'ros/pipeline'
import { PipelineContext } from 'providers/PipelineProvider'
import { PipelineNode } from './PipelineNode'

export const PreprocessorNode: React.FC = () => {
  const { preprocessorEnabled, preprocessorModule, preprocessorList } = useContext(PipelineContext)

  const handleToggle = (next: boolean) => {
    setPreprocessorEnabledRos(next, () => {
      console.log('Preprocessor ' + (next ? 'enabled' : 'disabled'))
    })
  }

  const handleModuleChange = (nextModule: string) => {
    setPreprocessorModuleRos(nextModule, () => {
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

