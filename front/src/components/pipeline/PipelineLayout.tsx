import React from 'react'
import styled from 'styled-components'

const Layout = styled.div`
  display: flex;
  flex-direction: column;
  gap: 16px;
`

export const PipelineLayout: React.FC<React.PropsWithChildren> = ({ children }) => {
  return <Layout>{children}</Layout>
}

