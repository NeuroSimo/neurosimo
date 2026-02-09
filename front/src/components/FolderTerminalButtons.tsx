import React, { useContext } from 'react'
import styled from 'styled-components'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'
import { faFolderOpen, faTerminal } from '@fortawesome/free-solid-svg-icons'
import { useGlobalConfig } from 'providers/GlobalConfigProvider'

const ButtonGroup = styled.div`
  display: flex;
  gap: 5px;
`

const IconButton = styled.button<{ disabled: boolean; $size?: number }>`
  background: none;
  border: 0.5px solid #666666;
  border-radius: 3px;
  width: ${props => props.$size || 22}px;
  height: ${props => props.$size || 22}px;
  cursor: ${props => props.disabled ? 'not-allowed' : 'pointer'};
  display: flex;
  align-items: center;
  justify-content: center;
  color: #666666;
  opacity: ${props => props.disabled ? 0.5 : 1};
  margin-left: 0px;

  &:hover {
    background-color: ${props => props.disabled ? 'transparent' : '#f0f0f0'};
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
      <IconButton
        onClick={handleOpenFolder}
        disabled={isDisabled}
        title={folderTitle || defaultFolderTitle}
        $size={size}
      >
        <FontAwesomeIcon icon={faFolderOpen} />
      </IconButton>
      <IconButton
        onClick={handleOpenTerminal}
        disabled={isDisabled}
        title={terminalTitle || defaultTerminalTitle}
        $size={size}
      >
        <FontAwesomeIcon icon={faTerminal} />
      </IconButton>
    </ButtonGroup>
  )
}
