import React, { useContext, useEffect, useState } from 'react'
import styled from 'styled-components'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'
import { faPlus } from '@fortawesome/free-solid-svg-icons'

import { StyledPanel, SmallerTitle, ConfigRow, ConfigLabel, Select, CONFIG_PANEL_WIDTH } from 'styles/General'
import { useParameters } from 'providers/ParameterProvider'
import { PipelineContext } from 'providers/PipelineProvider'
import { useSession, SessionStage } from 'providers/SessionProvider'
import { CommittableTextInput } from 'components/CommittableTextInput'
import { CommittableNumericInput } from 'components/CommittableNumericInput'
import { CreateProjectModal } from 'components/CreateProjectModal'
import { listProjects, setActiveProject } from 'ros/project'
import { ProjectContext } from 'providers/ProjectProvider'

const Container = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH}px;
  position: relative;
  margin-top: 0;
  margin-left: 0;
  left: 0;
`

export const ExperimentPanel: React.FC = () => {
  const { protocolName, protocolList } = useContext(PipelineContext)
  const { metadata, setExperimentProtocol, setSubjectId, setNotes } = useParameters()
  const { sessionState } = useSession()
  const { activeProject } = useContext(ProjectContext)
  const [projects, setProjects] = useState<string[]>([])
  const [isCreateModalOpen, setIsCreateModalOpen] = useState(false)

  const isSessionRunning = sessionState.stage !== SessionStage.STOPPED

  useEffect(() => {
    listProjects(setProjects)
  }, [])

  const refreshProjects = () => {
    listProjects(setProjects)
  }

  const handleProtocolChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const protocol = event.target.value
    setExperimentProtocol(protocol, () => {
      console.log('Protocol set to ' + protocol)
    })
  }

  const handleSubjectIdCommit = (value: string) => {
    setSubjectId(value, () => {
      console.log('Subject ID set to ' + value)
    })
  }

  const handleNotesCommit = (value: string) => {
    setNotes(value, () => {
      console.log('Notes set to ' + value)
    })
  }

  const handleProjectChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const newActiveProject = event.target.value
    setActiveProject(newActiveProject, () => {
      console.log('Active project set to ' + newActiveProject)
    })
  }

  return (
    <Container>
      <SmallerTitle>Experiment</SmallerTitle>
      <ConfigRow>
        <ConfigLabel>Project:</ConfigLabel>
        <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
          <button
            onClick={() => setIsCreateModalOpen(true)}
            disabled={isSessionRunning}
            style={{
              background: 'none',
              border: '0.5px solid #666666',
              borderRadius: '3px',
              width: '18px',
              height: '18px',
              cursor: isSessionRunning ? 'not-allowed' : 'pointer',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              color: '#666666',
              opacity: isSessionRunning ? 0.5 : 1,
            }}
            title="Create new project"
          >
            <FontAwesomeIcon icon={faPlus} />
          </button>
          <Select onChange={handleProjectChange} value={activeProject} disabled={isSessionRunning}>
            {projects.map((project, index) => (
              <option key={index} value={project}>
                {project}
              </option>
            ))}
          </Select>
        </div>
      </ConfigRow>
      <ConfigRow>
        <ConfigLabel>Subject ID:</ConfigLabel>
        <CommittableNumericInput
          value={metadata.subject_id}
          onCommit={handleSubjectIdCommit}
          prefix="S"
          maxLength={3}
          placeholder="001"
          disabled={isSessionRunning}
          width="20px"
        />
      </ConfigRow>
      <ConfigRow>
        <ConfigLabel>Notes:</ConfigLabel>
        <CommittableTextInput
          value={metadata.notes}
          onCommit={handleNotesCommit}
          placeholder="Enter notes"
          disabled={isSessionRunning}
          multiline={true}
          width="315px"
        />
      </ConfigRow>
      <ConfigRow>
        <ConfigLabel>Protocol:</ConfigLabel>
        <Select onChange={handleProtocolChange} value={protocolName} disabled={isSessionRunning}>
          {protocolList.map((protocol, index) => (
            <option key={index} value={protocol}>
              {protocol}
            </option>
          ))}
        </Select>
      </ConfigRow>

      <CreateProjectModal
        isOpen={isCreateModalOpen}
        onClose={() => setIsCreateModalOpen(false)}
        onProjectCreated={refreshProjects}
      />
    </Container>
  )
}

