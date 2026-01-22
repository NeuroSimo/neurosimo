import React, { useContext } from 'react'

import { useParameters } from 'providers/ParameterProvider'
import { PipelineContext } from 'providers/PipelineProvider'
import { PipelineNode } from './PipelineNode'

export const PreprocessorNode: React.FC = () => {
  const { preprocessorEnabled, preprocessorModule, preprocessorList } = useContext(PipelineContext)
  const { setPreprocessorEnabled, setPreprocessorModule } = useParameters()

  const handleToggle = (next: boolean) => {
    setPreprocessorEnabled(next, () => {
      console.log('Preprocessor ' + (next ? 'enabled' : 'disabled'))
    })
  }

  const handleModuleChange = (nextModule: string) => {
    setPreprocessorModule(nextModule, () => {
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

