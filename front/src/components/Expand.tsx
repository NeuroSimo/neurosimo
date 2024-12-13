import React from 'react'
import { ReactComponent as ArrowDown } from 'assets/arrow-down.svg'
import { ReactComponent as ArrowLeft } from 'assets/arrow-left.svg'

import styled from 'styled-components'

interface Props {
  expanded: boolean
  onClick: (event: any) => any
}

const Expand = ({ expanded, onClick }: Props) => {
  return <ExpandButton onClick={onClick}>{expanded ? <StyledArrowDown /> : <StyledArrowLeft />}</ExpandButton>
}

const ExpandButton = styled.button`
  all: unset;
  display: block;
  margin-left: auto;
  margin-right: 0;
`

const StyledArrowDown = styled(ArrowDown)`
  width: 15px;
  height: 15px;
  vertical-align: center;
  text-align: center;
  position: relative;
  top: 50%;
  -webkit-transform: translateY(-50%);
  -ms-transform: translateY(-50%);
  transform: translateY(-50%);
`

const StyledArrowLeft = styled(ArrowLeft)`
  width: 15px;
  height: 15px;
  vertical-align: center;
  text-align: center;
  position: relative;
  top: 50%;
  -webkit-transform: translateY(-50%);
  -ms-transform: translateY(-50%);
  transform: translateY(-50%);
`

export default Expand
