import React, { useContext } from 'react'
import styled from 'styled-components'

import { StyledPanel, SmallerTitle, ConfigRow, ConfigLabel, Select } from 'styles/General'
import { ToggleSwitch } from 'components/ToggleSwitch'
import { setDeciderEnabledRos, setDeciderModuleRos } from 'ros/pipeline'
import { PipelineContext } from 'providers/PipelineProvider'

const Container = styled(StyledPanel)`
  width: 154px;
  height: 92px;
  background-color: #e6ebf2;
  box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.8), 0 3px 10px rgba(0, 0, 0, 0.14);
`

export const DeciderNode: React.FC = () => {
  const { deciderEnabled: enabled, deciderModule: module, deciderList: modules, experimentState } = useContext(PipelineContext)

  const isExperimentOngoing = experimentState?.ongoing ?? false

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

