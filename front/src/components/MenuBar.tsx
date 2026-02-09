import React, { useState } from 'react'
import styled from 'styled-components'
import { GlobalConfigModal } from './GlobalConfigModal'

const MenuBarContainer = styled.div`
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  height: 30px;
  background-color: #f5f5f5;
  border-bottom: 1px solid #ddd;
  display: flex;
  align-items: center;
  padding: 0 8px;
  z-index: 1001;
  font-size: 14px;
`

const MenuButton = styled.button<{ isOpen?: boolean }>`
  background: ${props => props.isOpen ? '#e0e0e0' : 'transparent'};
  border: none;
  padding: 4px 12px;
  cursor: pointer;
  font-size: 14px;
  border-radius: 3px;
  
  &:hover {
    background-color: #e0e0e0;
  }
`

const DropdownMenu = styled.div`
  position: absolute;
  top: 28px;
  left: 8px;
  background: white;
  border: 1px solid #ddd;
  border-radius: 4px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.15);
  min-width: 180px;
  z-index: 1002;
`

const MenuItem = styled.button`
  display: block;
  width: 100%;
  padding: 8px 16px;
  text-align: left;
  background: none;
  border: none;
  cursor: pointer;
  font-size: 14px;
  
  &:hover {
    background-color: #f0f0f0;
  }
  
  &:first-child {
    border-radius: 4px 4px 0 0;
  }
  
  &:last-child {
    border-radius: 0 0 4px 4px;
  }
`

export const MenuBar: React.FC = () => {
  const [isFileMenuOpen, setIsFileMenuOpen] = useState(false)
  const [isGlobalConfigModalOpen, setIsGlobalConfigModalOpen] = useState(false)

  const handleFileMenuClick = () => {
    setIsFileMenuOpen(!isFileMenuOpen)
  }

  const handleGlobalConfigClick = () => {
    setIsGlobalConfigModalOpen(true)
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
            <MenuItem onClick={handleGlobalConfigClick}>
              Global Configuration...
            </MenuItem>
          </DropdownMenu>
        )}
      </MenuBarContainer>
      
      <GlobalConfigModal
        isOpen={isGlobalConfigModalOpen}
        onClose={() => setIsGlobalConfigModalOpen(false)}
      />
    </>
  )
}
