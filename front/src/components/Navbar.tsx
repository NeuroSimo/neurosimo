import React from 'react'
import styled from 'styled-components'
import { useLocation, useNavigate } from 'react-router-dom'

export const Navbar = () => {
  const navigate = useNavigate()
  const location = useLocation()

  const goToUrl = (url: string) => {
    navigate(url, { replace: true })
  }

  return (
    <Nav>
      <TabButton selected={location.pathname === '/'} onClick={() => goToUrl('/')}>
        Targets table
      </TabButton>
      <TabButton selected={location.pathname === '/session'} onClick={() => goToUrl('/session')}>
        Session
      </TabButton>
      <TabButton selected={location.pathname === '/plot'} onClick={() => goToUrl('/plot')}>
        Plot
      </TabButton>
    </Nav>
  )
}

const TabButton = styled.button<{
  selected: boolean
}>`
  all: unset;
  background-color: ${(p) => (p.selected ? p.theme.colors.lightergray : p.theme.colors.lightgray)};
  padding: 0.3rem;
  border: 1px solid ${(p) => p.theme.colors.gray};

  :hover {
    background-color: ${(p) => p.theme.colors.gray};
  }
`

const Nav = styled.nav`
  margin: 0;
  padding-left: 0.5em;
  box-sizing: border-box;
`

const Ul = styled.ul`
  list-style-type: none;
  margin: 0;
  padding: 0;
  display: flex;
  flex-direction: row;
`

const ListElement = styled.li`
  cursor: pointer;
  font-size: ${(p) => p.theme.typography.medium};
  padding-right: 0.5em;
  padding-left: 0.5em;
  box-shadow: inset 0 -1px 0 ${(p) => p.theme.colors.gray};

  background-color: ${(p) => p.theme.colors.lightergray};

  :hover {
    background-color: ${(p) => p.theme.colors.lightgray};
  }
`
