import React from 'react'
import { Color } from '../styles/theme'
import styled from 'styled-components'

interface RectangleProps {
  color: Color
}

const Rectangle = ({ color }: RectangleProps) => {
  return (
    <Container>
      <RectangleDiv color={color} />
    </Container>
  )
}

const Container = styled.div`
  display: flex;
  justify-content: center;
  align-items: center;
`

const RectangleDiv = styled.div<{
  color: Color
}>`
  background: ${(p) => p.color};
  width: 1rem;
  height: 1rem;
  margin-right: 0.2rem;
`

export default Rectangle
