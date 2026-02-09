import React, { useContext, useEffect, useState } from 'react'
import styled from 'styled-components'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'
import { faPlus, faInfoCircle } from '@fortawesome/free-solid-svg-icons'

import { StyledPanel, SmallerTitle, ConfigRow, ConfigLabel, Select, CONFIG_PANEL_WIDTH } from 'styles/General'
import { useSessionConfig } from 'providers/SessionConfigProvider'
import { ModuleListContext } from 'providers/ModuleListProvider'
import { useSession, SessionStateValue } from 'providers/SessionProvider'
import { CommittableTextInput } from 'components/CommittableTextInput'
import { CommittableNumericInput } from 'components/CommittableNumericInput'
import { CreateProjectModal } from 'components/CreateProjectModal'
import { ProtocolInfoModal } from 'components/ProtocolInfoModal'
import { FolderTerminalButtons } from 'components/FolderTerminalButtons'
import { listProjects, setActiveProject } from 'ros/project'
import { useGlobalConfig } from 'providers/GlobalConfigProvider'
import { getProtocolInfoRos, ProtocolInfo } from 'ros/experiment'

const Container = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH}px;
  position: relative;
  margin-top: 0;
  margin-left: 0;
  left: 0;
`

const IconButtonWrapper = styled.div`
  display: flex;
  align-items: center;
  gap: 8px;
`

const IconButton = styled.button<{ disabled: boolean }>`
  background: none;
  border: 0.5px solid #666666;
  border-radius: 3px;
  width: 22px;
  height: 22px;
  cursor: ${props => props.disabled ? 'not-allowed' : 'pointer'};
  display: flex;
  align-items: center;
  justify-content: center;
  color: #666666;
  opacity: ${props => props.disabled ? 0.5 : 1};
`

const ProtocolSelect = styled(Select)`
  max-width: 150px;
`

export const ExperimentPanel: React.FC = () => {
  const { protocolName, protocolList } = useContext(ModuleListContext)
  const { metadata, setExperimentProtocol, setSubjectId, setNotes } = useSessionConfig()
  const { sessionState } = useSession()
  const { activeProject } = useGlobalConfig()
  const [projects, setProjects] = useState<string[]>([])
  const [isCreateModalOpen, setIsCreateModalOpen] = useState(false)
  const [isProtocolInfoModalOpen, setIsProtocolInfoModalOpen] = useState(false)
  const [protocolInfo, setProtocolInfo] = useState<ProtocolInfo | null>(null)

  const isSessionRunning = sessionState.state === SessionStateValue.RUNNING
  const isElectron = !!(window as any).electronAPI

  useEffect(() => {
    listProjects(setProjects)
  }, [])

  const handleProjectCreated = (projectName: string) => {
    listProjects(setProjects)
    setActiveProject(projectName, () => {
      console.log('Switched to newly created project:', projectName)
    })
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

  const handleProtocolInfo = () => {
    if (!protocolName || protocolName.trim() === '' || !activeProject) return
    
    getProtocolInfoRos(activeProject, protocolName, (info) => {
      if (!info) {
        console.error('Failed to get protocol info for:', protocolName)
        return
      }
      setProtocolInfo(info)
      setIsProtocolInfoModalOpen(true)
    })
  }

  return (
    <Container>
      <SmallerTitle>Experiment</SmallerTitle>
      <ConfigRow>
        <ConfigLabel>Project:</ConfigLabel>
        <IconButtonWrapper>
          <IconButton
            onClick={() => setIsCreateModalOpen(true)}
            disabled={isSessionRunning}
            title="Create new project"
          >
            <FontAwesomeIcon icon={faPlus} />
          </IconButton>
          <Select onChange={handleProjectChange} value={activeProject} disabled={isSessionRunning}>
            {projects.map((project, index) => (
              <option key={index} value={project}>
                {project}
              </option>
            ))}
          </Select>
        </IconButtonWrapper>
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
        <IconButtonWrapper>
          <FolderTerminalButtons folderName="protocols" />
          <IconButton
            onClick={handleProtocolInfo}
            disabled={!protocolName || protocolName.trim() === '' || !activeProject}
            title="Show protocol info"
          >
            <FontAwesomeIcon icon={faInfoCircle} />
          </IconButton>
          <ProtocolSelect onChange={handleProtocolChange} value={protocolName} disabled={isSessionRunning}>
            {protocolList.map((protocol, index) => (
              <option key={index} value={protocol}>
                {protocol}
              </option>
            ))}
          </ProtocolSelect>
        </IconButtonWrapper>
      </ConfigRow>

      <CreateProjectModal
        isOpen={isCreateModalOpen}
        onClose={() => setIsCreateModalOpen(false)}
        onProjectCreated={handleProjectCreated}
      />

      <ProtocolInfoModal
        isOpen={isProtocolInfoModalOpen}
        onClose={() => setIsProtocolInfoModalOpen(false)}
        protocolInfo={protocolInfo}
      />
    </Container>
  )
}

