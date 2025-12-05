import React from 'react'
import styled from 'styled-components'

import { StyledPanel, SmallerTitle, ConfigRow, ConfigLabel, Select } from 'styles/General'
import { ToggleSwitch } from 'components/ToggleSwitch'
import { setPreprocessorEnabledRos, setPreprocessorModuleRos } from 'ros/pipeline'

type Props = {
  enabled: boolean
  module: string
  modules: string[]
}

const Container = styled(StyledPanel)`
  width: 154px;
  height: 92px;
`

export const PreprocessorNode: React.FC<Props> = ({ enabled, module, modules }) => {
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
        <ToggleSwitch type='flat' checked={enabled} onChange={handleToggle} />
      </ConfigRow>
      <ConfigRow>
        <ConfigLabel>Module:</ConfigLabel>
        <Select onChange={handleModuleChange} value={module}>
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

