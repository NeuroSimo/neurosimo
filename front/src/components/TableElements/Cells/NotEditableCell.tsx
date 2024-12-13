import React from 'react'
import styled from 'styled-components'
import { CellProps } from 'types/table'

const NotEditableCell = ({ value: initialValue }: CellProps) => {
  return <DisabledInput value={initialValue} disabled={true} />
}
const DisabledInput = styled.input`
  width: 100%;
  color: ${(p) => p.theme.colors.primary};
  border: 0;
  background-color: inherit;
  font-size: 1rem;
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Roboto', 'Oxygen', 'Ubuntu', 'Cantarell', 'Fira Sans',
    'Droid Sans', 'Helvetica Neue', sans-serif;
`

export default NotEditableCell
