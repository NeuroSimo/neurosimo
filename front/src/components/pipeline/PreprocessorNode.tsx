import React, { useContext } from 'react'
import styled from 'styled-components'

import { StyledPanel, SmallerTitle, ConfigRow, ConfigLabel, Select } from 'styles/General'
import { ToggleSwitch } from 'components/ToggleSwitch'
import { setPreprocessorEnabledRos, setPreprocessorModuleRos } from 'ros/pipeline'
import { PipelineContext } from 'providers/PipelineProvider'

const Container = styled(StyledPanel)`
  width: 154px;
  height: 92px;
  background-color: #e6ebf2;
  box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.8), 0 3px 10px rgba(0, 0, 0, 0.14);
`

export const PreprocessorNode: React.FC = () => {
  const { preprocessorEnabled: enabled, preprocessorModule: module, preprocessorList: modules, experimentState } = useContext(PipelineContext)

  const isExperimentOngoing = experimentState?.ongoing ?? false

  const handleToggle = (next: boolean) => {
    setPreprocessorEnabledRos(next, () => {
      console.log('Preprocessor ' + (next ? 'enabled' : 'disabled'))
    })
  }

  const handleModuleChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const nextModule = event.target.value
    setPreprocessorModuleRos(nextModule, () => {
      console.log('Preprocessor set to ' + nextModule)
    })
  }

  return (
    <Container>
      <SmallerTitle>Preprocessor {enabled ? '' : '(bypass)'}</SmallerTitle>
      <ConfigRow>
        <ConfigLabel>Enabled:</ConfigLabel>
        <ToggleSwitch type='flat' checked={enabled} onChange={handleToggle} disabled={isExperimentOngoing} />
      </ConfigRow>
      <ConfigRow>
        <ConfigLabel>Module:</ConfigLabel>
        <Select onChange={handleModuleChange} value={module} disabled={isExperimentOngoing}>
          {modules.map((mod, index) => (
            <option key={index} value={mod}>
              {mod}
            </option>
          ))}
        </Select>
      </ConfigRow>
    </Container>
  )
}

