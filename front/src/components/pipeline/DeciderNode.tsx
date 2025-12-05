import React from 'react'
import styled from 'styled-components'

import { StyledPanel, SmallerTitle, ConfigRow, ConfigLabel, Select } from 'styles/General'
import { ToggleSwitch } from 'components/ToggleSwitch'

type Props = {
  enabled: boolean
  module: string
  modules: string[]
  onToggle: (enabled: boolean) => void
  onModuleChange: (event: React.ChangeEvent<HTMLSelectElement>) => void
}

const Container = styled(StyledPanel)`
  width: 154px;
  height: 92px;
`

export const DeciderNode: React.FC<Props> = ({ enabled, module, modules, onToggle, onModuleChange }) => {
  return (
    <Container>
      <SmallerTitle>Decider</SmallerTitle>
      <ConfigRow>
        <ConfigLabel>Enabled:</ConfigLabel>
        <ToggleSwitch type='flat' checked={enabled} onChange={onToggle} />
      </ConfigRow>
      <ConfigRow>
        <ConfigLabel>Module:</ConfigLabel>
        <Select onChange={onModuleChange} value={module}>
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

