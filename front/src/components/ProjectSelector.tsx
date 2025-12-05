import React from 'react'
import styled from 'styled-components'

import { ProjectRow, Select } from 'styles/General'

type Props = {
  projects: string[]
  activeProject: string
  onChange: (event: React.ChangeEvent<HTMLSelectElement>) => void
}

const Label = styled.label`
  width: 92px;
  text-align: left;
  margin-right: 5px;
  margin-left: 18px;
  display: inline-block;
`

export const ProjectSelector: React.FC<Props> = ({ projects, activeProject, onChange }) => {
  return (
    <ProjectRow>
      <Label>Project:</Label>
      <Select onChange={onChange} value={activeProject}>
        {projects.map((project, index) => (
          <option key={index} value={project}>
            {project}
          </option>
        ))}
      </Select>
    </ProjectRow>
  )
}

