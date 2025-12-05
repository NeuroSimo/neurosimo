import React, { useContext, useEffect, useState } from 'react'
import styled from 'styled-components'

import { ProjectRow, Select, StyledPanel, SmallerTitle, CONFIG_PANEL_WIDTH } from 'styles/General'
import { listProjects, setActiveProject } from 'ros/project'
import { ProjectContext } from 'providers/ProjectProvider'

const Container = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH}px;
  position: relative;
`

const Label = styled.label`
  width: 92px;
  text-align: left;
  margin-right: 5px;
  margin-left: 0;
  display: inline-block;
`

export const ProjectSelector: React.FC = () => {
  const { activeProject } = useContext(ProjectContext)
  const [projects, setProjects] = useState<string[]>([])

  useEffect(() => {
    listProjects(setProjects)
  }, [])

  const handleChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const newActiveProject = event.target.value
    setActiveProject(newActiveProject, () => {
      console.log('Active project set to ' + newActiveProject)
    })
  }

  return (
    <Container>
      <SmallerTitle>Project</SmallerTitle>
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
    </Container>
  )
}

