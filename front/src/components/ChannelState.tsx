import React from 'react'
import styled from 'styled-components'

import { ChannelState as ChannelStateType } from 'providers/SystemProvider'
import { getTrueKeys } from '../utils'

export const ChannelState = (state: ChannelStateType) => {
  const getListValue = (object: any) => {
    const keys: string[] = getTrueKeys(object)

    if (keys.length > 0) {
      return keys.map((key) => {
        return <span key={key}>{key}</span>
      })
    } else {
      /* No errors, do not display anything. */
      return <span></span>
    }
  }

  return (
    <tr>
      <Td>{state.channel_index + 1}</Td>
      <Td>{state.voltage}</Td>
      {/*<Td>{state.temperature}</Td>*/}
      <Td>{state.pulse_count}</Td>
      <Td>{getListValue(state.channel_error)}</Td>
    </tr>
  )
}

const Td = styled.td`
  text-align: right; /* Right align for the data cells */
  /* ... other styles if any ... */
`
