import React from 'react'
import styled from 'styled-components'

import { StyledPanel, SmallerTitle, ConfigRow, ConfigLabel } from 'styles/General'

type Props = {
  typeLabel?: string
}

const Container = styled(StyledPanel)`
  width: 154px;
  height: 55px;
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

