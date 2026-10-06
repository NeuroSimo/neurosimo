import React, { useState } from 'react'
import styled from 'styled-components'
import { SystemConfigModal } from './SystemConfigModal'
import { palette } from 'styles/General'

const MenuBarContainer = styled.div`
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  height: 30px;
  box-sizing: border-box;
  background-color: ${palette.surface};
  border-bottom: 1px solid ${palette.border};
  display: flex;
  align-items: center;
  padding: 0 6px;
  z-index: 1001;
  font-size: 13px;
`

const MenuButton = styled.button<{ isOpen?: boolean }>`
  background: ${props => props.isOpen ? palette.surfaceHover : 'transparent'};
  border: none;
  padding: 3px 10px;
  cursor: pointer;
  font-size: 13px;
  font-family: inherit;
  color: ${palette.text};
  border-radius: 3px;
  
  &:hover {
    background-color: ${palette.surfaceHover};
  }
`

const DropdownMenu = styled.div`
  position: absolute;
  top: 28px;
  left: 6px;
  background: ${palette.surfaceRaised};
  border: 1px solid ${palette.borderStrong};
  border-radius: 3px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.4);
  min-width: 180px;
  padding: 3px 0;
  z-index: 1002;
`

const MenuItem = styled.button`
  display: block;
  width: 100%;
  padding: 5px 14px;
  text-align: left;
  background: none;
  border: none;
  cursor: pointer;
  font-size: 13px;
  font-family: inherit;
  color: ${palette.text};
  
  &:hover {
    background-color: ${palette.surfaceHover};
  }
`

export const MenuBar: React.FC = () => {
  const [isFileMenuOpen, setIsFileMenuOpen] = useState(false)
  const [isSystemConfigModalOpen, setIsSystemConfigModalOpen] = useState(false)

  const handleFileMenuClick = () => {
    setIsFileMenuOpen(!isFileMenuOpen)
  }

  const handleSettingsClick = () => {
    setIsSystemConfigModalOpen(true)
    setIsFileMenuOpen(false)
  }

  // Close menu when clicking outside
  React.useEffect(() => {
    if (!isFileMenuOpen) return

    const handleClickOutside = () => {
      setIsFileMenuOpen(false)
    }

    document.addEventListener('click', handleClickOutside)
    return () => document.removeEventListener('click', handleClickOutside)
  }, [isFileMenuOpen])

  return (
    <>
      <MenuBarContainer>
        <MenuButton
          isOpen={isFileMenuOpen}
          onClick={(e) => {
            e.stopPropagation()
            handleFileMenuClick()
          }}
        >
          File
        </MenuButton>
        {isFileMenuOpen && (
          <DropdownMenu onClick={(e) => e.stopPropagation()}>
            <MenuItem onClick={handleSettingsClick}>
              Settings...
            </MenuItem>
          </DropdownMenu>
        )}
      </MenuBarContainer>
      
      <SystemConfigModal
        isOpen={isSystemConfigModalOpen}
        onClose={() => setIsSystemConfigModalOpen(false)}
      />
    </>
  )
}
