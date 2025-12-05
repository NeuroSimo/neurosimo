import React from 'react'
import styled from 'styled-components'

import { StyledPanel, SmallerTitle, ConfigRow, ConfigLabel, Select } from 'styles/General'
import { ToggleSwitch } from 'components/ToggleSwitch'
import { setDeciderEnabledRos, setDeciderModuleRos } from 'ros/pipeline'

type Props = {
  enabled: boolean
  module: string
  modules: string[]
}

const Container = styled(StyledPanel)`
  width: 154px;
  height: 92px;
`

export const DeciderNode: React.FC<Props> = ({ enabled, module, modules }) => {
  const handleToggle = (next: boolean) => {
    setDeciderEnabledRos(next, () => {
      console.log('Decider ' + (next ? 'enabled' : 'disabled'))
    })
  }

  const handleModuleChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const nextModule = event.target.value
    setDeciderModuleRos(nextModule, () => {
      console.log('Decider set to ' + nextModule)
    })
  }

  return (
    <Container>
      <SmallerTitle>Decider</SmallerTitle>
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

