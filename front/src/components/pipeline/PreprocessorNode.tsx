import React, { useContext } from 'react'

import { useSessionConfig } from 'providers/SessionConfigProvider'
import { PipelineConfigContext } from 'providers/PipelineConfigProvider'
import { PipelineNode } from './PipelineNode'

export const PreprocessorNode: React.FC = () => {
  const { preprocessorEnabled, preprocessorModule, preprocessorList } = useContext(PipelineConfigContext)
  const { setPreprocessorEnabled, setPreprocessorModule } = useSessionConfig()

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
      folderName="preprocessor"
    />
  )
}

