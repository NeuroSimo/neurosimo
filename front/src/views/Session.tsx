import React, { useState } from 'react'
import styled from 'styled-components'
import ROSLIB from 'roslib'

import PulseSequenceConfiguration from './PulseSequenceConfiguration'
import { ISession, SessionMessage, Train } from '../types/pulseSequence'
import { objectKeysToSnakeCase } from '../utils'
import { useAppDispatch, useAppSelector } from 'providers/reduxHooks'
import { setDescription } from 'reducers/sessionReducer'
import { startSessionService } from 'ros/services/session'

const Session = () => {
  const { description, channels, iti, ibi, nofBurstsInTrains, nofTrains } = useAppSelector((state) => state.session)

  const { sequences } = useAppSelector((state) => state.sequences)
  const dispatch = useAppDispatch()

  const [statusMessage, setStatusMessage] = useState<string>('')
  const [status, setStatus] = useState<'OK' | 'ERROR'>('OK')

  const startSequence = (event: any) => {
    setStatus('OK')
    setStatusMessage('Starting sequence...')

    const channelData = channels
      .filter((channel) => channel.enabled)
      .map((fullChannel) => {
        const { enabled, ...channel } = fullChannel
        return channel
      })

    const train: Train = {
      ibi,
      sequences,
      nofBursts: nofBurstsInTrains,
    }

    const session: ISession = {
      iti,
      description,
      nofTrains,
      train,
    }
    const messageData: SessionMessage = {
      session,
    }

    console.log('starting sequence:', messageData)

    const messageDataSnakeCase = objectKeysToSnakeCase(messageData)

    const message = new ROSLIB.Message(messageDataSnakeCase)
    const request = new ROSLIB.ServiceRequest({
      session: message,
    })

    startSessionService.callService(
      request,
      (response) => {
        if (!response.success) {
          console.error('FAILED TO START SEQUENCE, response', response)
          setStatus('ERROR')
          setStatusMessage(`Failed to start sequence! Error: ${response.status}`)
        } else {
          console.log('Started sequence')
          setStatus('OK')
          setStatusMessage('Started sequence')
        }
      },
      (error) => {
        console.error(error)
        setStatus('ERROR')
        setStatusMessage(`Failed to start sequence! Error: ${error}`)
      },
    )
  }

  return (
    <div>
      <h1>Session</h1>
      <label htmlFor='description'>Description </label>
      <input
        name='description'
        type='text'
        value={description}
        onChange={(event) => dispatch(setDescription(event.target.value))}
      />

      <br />

      <PulseSequenceConfiguration />

      <button onClick={startSequence}>Start sequence</button>
      <div>
        <StatusMessage status={status}>{statusMessage}</StatusMessage>
      </div>
    </div>
  )
}

const StatusMessage = styled.p<{
  status: 'OK' | 'ERROR'
}>`
  color: ${(p) => (p.status === 'OK' ? p.theme.colors.primary : p.theme.colors.error)};
`

export default Session
