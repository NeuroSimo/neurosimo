import React from 'react'
import styled from 'styled-components'

type Props = {
  setup: React.ReactNode
  pipeline: React.ReactNode
}

const Layout = styled.div`
  display: grid;
  grid-template-columns: auto 1fr;
  grid-template-rows: auto 1fr;
  gap: 16px 24px;
  align-items: start;
`

const SetupArea = styled.div`
  display: flex;
  flex-direction: column;
  gap: 12px;
  grid-column: 1;
  grid-row: 1 / span 2;
`

const PipelineArea = styled.div`
  display: flex;
  flex-direction: column;
  gap: 12px;
  grid-column: 2;
  grid-row: 1 / span 2;
`

export const PipelineLayout: React.FC<Props> = ({ setup, pipeline }) => {
  return (
    <Layout>
      <SetupArea>{setup}</SetupArea>
      <PipelineArea>{pipeline}</PipelineArea>
    </Layout>
  )
}

