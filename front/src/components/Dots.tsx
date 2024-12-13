import React from 'react'
import { ReactComponent as DotsIcon } from 'assets/dots.svg'
import styled from 'styled-components'

const Dots = () => {
  return <StyledDots />
}

const StyledDots = styled(DotsIcon)`
  width: 20px;
  height: 20px;
  vertical-align: center;
  text-align: center;
`

export default Dots
