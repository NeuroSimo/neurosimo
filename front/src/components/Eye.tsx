import React from 'react'
import { ReactComponent as EyeIcon } from 'assets/eye.svg'
import { ReactComponent as EyeSlashIcon } from 'assets/eye-slash.svg'
import styled from 'styled-components'

interface Props {
  visible: boolean
}

const Eye = ({ visible }: Props) => {
  return visible ? <StyledEye /> : <StyledEyeSlash />
}
const StyledEye = styled(EyeIcon)`
  width: 20px;
  height: 20px;
  vertical-align: center;
  text-align: center;
`

const StyledEyeSlash = styled(EyeSlashIcon)`
  width: 20px;
  height: 20px;
  vertical-align: center;
  text-align: center;
`

export default Eye
