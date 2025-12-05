import React, { useContext } from 'react'
import styled from 'styled-components'

import { StyledPanel, SmallerTitle, ConfigRow, ConfigLabel, Select } from 'styles/General'
import { ToggleSwitch } from 'components/ToggleSwitch'
import { setPresenterEnabledRos, setPresenterModuleRos } from 'ros/pipeline'
import { PipelineContext } from 'providers/PipelineProvider'

const Container = styled(StyledPanel)`
  width: 154px;
  height: 92px;
  background-color: #e6ebf2;
  box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.8), 0 3px 10px rgba(0, 0, 0, 0.14);
`

export const PresenterNode: React.FC = () => {
  const { presenterEnabled: enabled, presenterModule: module, presenterList: modules } = useContext(PipelineContext)

  const handleToggle = (next: boolean) => {
    setPresenterEnabledRos(next, () => {
      console.log('Presenter ' + (next ? 'enabled' : 'disabled'))
    })
  }

  const handleModuleChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const nextModule = event.target.value
    setPresenterModuleRos(nextModule, () => {
      console.log('Presenter set to ' + nextModule)
    })
  }

  return (
    <Container>
      <SmallerTitle>Presenter</SmallerTitle>
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

