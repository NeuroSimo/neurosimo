import React, { useContext } from 'react'
import styled from 'styled-components'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'
import { faFolderOpen, faTerminal } from '@fortawesome/free-solid-svg-icons'
import { useGlobalConfig } from 'providers/GlobalConfigProvider'

const ButtonGroup = styled.div`
  display: flex;
  gap: 5px;
`

const InfoIcon = styled.button<{ disabled: boolean; $size?: number }>`
  background: none;
  border: none;
  width: ${props => props.$size || 22}px;
  height: ${props => props.$size || 22}px;
  cursor: ${props => props.disabled ? 'not-allowed' : 'pointer'};
  display: flex;
  align-items: center;
  justify-content: center;
  color: #007bff;
  font-size: 16px;
  opacity: ${props => props.disabled ? 0.5 : 1};
  margin-left: 0px;
  transition: color 0.2s;

  &:hover {
    color: ${props => props.disabled ? '#007bff' : '#0056b3'};
  }
`

interface FolderTerminalButtonsProps {
  folderName: string
  disabled?: boolean
  folderTitle?: string
  terminalTitle?: string
  size?: number
}

export const FolderTerminalButtons: React.FC<FolderTerminalButtonsProps> = ({
  folderName,
  disabled = false,
  folderTitle,
  terminalTitle,
  size,
}) => {
  const { activeProject } = useGlobalConfig()
  const isElectron = !!(window as any).electronAPI

  const handleOpenFolder = async () => {
    if (!activeProject) return
    
    const error = await (window as any).electronAPI?.openProjectFolder(activeProject, folderName)
    if (error) console.error('Failed to open folder:', error)
  }

  const handleOpenTerminal = async () => {
    if (!activeProject) return
    
    const error = await (window as any).electronAPI?.openTerminalInFolder(activeProject, folderName)
    if (error) console.error('Failed to open terminal:', error)
  }

  const isDisabled = disabled || !activeProject || !isElectron

  const defaultFolderTitle = isElectron ? `Open ${folderName} folder` : "Only available in Electron"
  const defaultTerminalTitle = isElectron ? `Open terminal in ${folderName}` : "Only available in Electron"

  return (
    <ButtonGroup>
      <InfoIcon
        onClick={handleOpenFolder}
        disabled={isDisabled}
        title={folderTitle || defaultFolderTitle}
        $size={size}
      >
        <FontAwesomeIcon icon={faFolderOpen} />
      </InfoIcon>
      <InfoIcon
        onClick={handleOpenTerminal}
        disabled={isDisabled}
        title={terminalTitle || defaultTerminalTitle}
        $size={size}
      >
        <FontAwesomeIcon icon={faTerminal} />
      </InfoIcon>
    </ButtonGroup>
  )
}
