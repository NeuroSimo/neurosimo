import React, { useEffect, useState } from 'react'
import styled from 'styled-components'
import { Feedback } from 'types/event'
import { getKeyByValue } from 'utils'
import { errorsByType } from '../types/eventErrors'

type Props = {
  feedback: Feedback | undefined
}

export const EventFeedbacks = ({ feedback }: Props) => {
  const [successfulEvents, setSuccessfulEvents] = useState<Feedback[]>([])
  const [failedEvents, setFailedEvents] = useState<Feedback[]>([])

  useEffect(() => {
    if (!feedback) return

    if (feedback.error.value == 0) {
      setSuccessfulEvents([...successfulEvents, feedback])
    } else {
      setFailedEvents([...failedEvents, feedback])
    }
  }, [feedback])

  return (
    <div>
      <h3>Actions</h3>
      <h4>Successful</h4>
      <EventContainer>
        {successfulEvents.map((successfulEvent) => {
          return <p key={`successful-event-${successfulEvent.id}`}>{successfulEvent.id}</p>
        })}
      </EventContainer>
      <h4>Failed</h4>
      <EventContainer>
        {failedEvents.map((failedEvent) => {
          return (
            <p key={`failed-event-${failedEvent.id}`}>
              {failedEvent.id}, reason: {getKeyByValue(errorsByType[failedEvent.type], failedEvent.error.value)}
            </p>
          )
        })}
      </EventContainer>
    </div>
  )
}

const EventContainer = styled.div`
  max-height: 75px;
  overflow-y: auto;
  overflow-x: hidden;
`
