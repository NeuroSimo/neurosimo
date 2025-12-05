import React from 'react'
import styled from 'styled-components'

import { StyledPanel, SmallerTitle, ConfigRow, ConfigLabel } from 'styles/General'

type Props = {
  typeLabel?: string
}

const Container = styled(StyledPanel)`
  width: 154px;
  height: 55px;
  background-color: #e6ebf2;
  box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.8), 0 3px 10px rgba(0, 0, 0, 0.14);
`

export const TmsNode: React.FC<Props> = ({ typeLabel = 'Multi-locus' }) => {
  return (
    <Container>
      <SmallerTitle>TMS device</SmallerTitle>
      <ConfigRow>
        <ConfigLabel>Type:</ConfigLabel>
        <ConfigLabel>{typeLabel}</ConfigLabel>
      </ConfigRow>
    </Container>
  )
}

