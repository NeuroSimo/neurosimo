import React, { useCallback, useContext, useEffect, useState } from 'react'
import styled from 'styled-components'
import throttle from 'throttleit'

import { StyledPanel, StateRow, StateTitle, StateValue } from 'styles/General'

import { SessionControl } from 'components/SessionControl'

import { SessionContext, SessionState, HumanReadableSessionState } from 'providers/SessionProvider'

const getKeyByValue = (object: any, value: any) => {
  if (!object) {
    return null
  }
  return Object.keys(object).find((key) => object[key] === value)
}

const SessionPanel = styled(StyledPanel)`
  width: 185px;
  height: 110px;
  position: fixed;
  top: 274px;
  right: 4px;
  z-index: 1000;
`

export const SessionDisplay: React.FC = () => {
  const { session } = useContext(SessionContext)

  const [latestUpdate, setLatestUpdate] = useState<Date>()

  const throttledUpdate = useCallback(
    throttle(() => {
      setLatestUpdate(new Date())
    }, 1000),
    /* Empty dependency array ensures the function is retained across renders. */
    []
  )

  /* Session is updated once every 1 ms, hence the throttling.

    TODO: Consider having a version of session that is updated less frequently to avoid having to throttle here. */
  useEffect(() => {
    throttledUpdate()
  }, [session, throttledUpdate])

  const formatDate = (isoString: any) => {
    const date = new Date(isoString)

    const year = date.getUTCFullYear()
    const month = String(date.getUTCMonth() + 1).padStart(2, '0')
    const day = String(date.getUTCDate()).padStart(2, '0')

    const hours = String(date.getUTCHours()).padStart(2, '0')
    const minutes = String(date.getUTCMinutes()).padStart(2, '0')
    const seconds = String(date.getUTCSeconds()).padStart(2, '0')

    return `${year}-${month}-${day} ${hours}:${minutes}:${seconds}`
  }

  const getHumanReadableSessionState = (sessionState: any, value: any) => {
    const key = getKeyByValue(SessionState, value)
    if (key) {
      return HumanReadableSessionState[key as keyof typeof HumanReadableSessionState] || 'Unknown state'
    } else {
      return 'Unknown state'
    }
  }

  return (
    <SessionPanel>
      <StateRow>
        <StateTitle>Time:</StateTitle>
        <StateValue>{latestUpdate ? formatDate(latestUpdate.toISOString()) : ''}</StateValue>
      </StateRow>
      <br />
      <StateRow>
        <StateTitle>Session:</StateTitle>
        <StateValue>{getHumanReadableSessionState(SessionState, session?.state.value)}</StateValue>
      </StateRow>
      <StateRow>
        <StateTitle>Session time:</StateTitle>
        <StateValue>{session?.time.toFixed(1)} s</StateValue>
      </StateRow>
      <br />
      <SessionControl />
    </SessionPanel>
  )
}
