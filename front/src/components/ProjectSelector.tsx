import React from 'react'
import styled from 'styled-components'

import { ProjectRow, Select } from 'styles/General'
import { setActiveProject } from 'ros/project'

type Props = {
  projects: string[]
  activeProject: string
}

const Label = styled.label`
  width: 92px;
  text-align: left;
  margin-right: 5px;
  margin-left: 18px;
  display: inline-block;
`

export const ProjectSelector: React.FC<Props> = ({ projects, activeProject }) => {
  const handleChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const newActiveProject = event.target.value
    setActiveProject(newActiveProject, () => {
      console.log('Active project set to ' + newActiveProject)
    })
  }

  return (
    <ProjectRow>
      <Label>Project:</Label>
      <Select onChange={handleChange} value={activeProject}>
        {projects.map((project, index) => (
          <option key={index} value={project}>
            {project}
          </option>
        ))}
      </Select>
    </ProjectRow>
  )
}

