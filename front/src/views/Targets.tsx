import React, { useEffect, useState } from 'react'
import styled from 'styled-components'

import { useAppDispatch } from 'providers/reduxHooks'

import { EulerAngles, Position, PositionMessage, StateMessage } from 'types/target'
import { expand, objectKeysToCamelCase } from 'utils'

import SequenceTable from 'components/SequenceTable'
import TargetTable from 'components/TargetTable'
import { setTargets } from 'reducers/targetReducer'
import { setPulseSequences } from 'reducers/sequenceReducer'
import { addTargetToRos } from 'ros/services/target'

import { clearRosState, coilPositionSubscriber, plannerStateSubscriber } from 'ros/ros'

const Targets = () => {
  const [position, setPosition] = useState<Position>({
    x: 0,
    y: 0,
    z: 0,
  })
  const [orientation, setOrientation] = useState<EulerAngles>({
    alpha: 0,
    beta: 0,
    gamma: 0,
  })

  const [tab, setTab] = useState<'TARGETS' | 'SEQUENCES'>('TARGETS')

  const dispatch = useAppDispatch()

  const updateState = (message: StateMessage) => {
    const pulseSequences = objectKeysToCamelCase(message.pulse_sequences)
    const targets = objectKeysToCamelCase(message.targets)

    dispatch(setTargets(targets))
    dispatch(setPulseSequences(pulseSequences))
  }

  const updatePosition = (message: PositionMessage) => {
    setPosition(message.position)
    setOrientation(message.orientation)
  }

  useEffect(() => {
    plannerStateSubscriber.subscribe(updateState)
    coilPositionSubscriber.subscribe(updatePosition)
  }, [])

  const addTarget = () => {
    addTargetToRos(position, orientation)
  }

  const table = () => {
    switch (tab) {
      case 'TARGETS':
        return <TargetTable />
      case 'SEQUENCES':
        return <SequenceTable />
    }
  }

  const handleViewChangeClick = (event: any) => {
    const target = event.target.name
    setTab(target)
  }

  const handleResetStateClick = (event: any) => {
    clearRosState()
  }

  return (
    <>
      <TabButton name='TARGETS' onClick={handleViewChangeClick} selected={tab === 'TARGETS'} id='targets-view-button'>
        Targets
      </TabButton>
      <TabButton
        name='SEQUENCES'
        onClick={handleViewChangeClick}
        selected={tab === 'SEQUENCES'}
        id='sequences-view-button'
      >
        Sequences
      </TabButton>

      <br />

      {table()}

      <p>Current position: {expand(position)}</p>
      <p>Current orientation: {expand(orientation)}</p>
      <button onClick={addTarget} id='add-target-button'>
        Add current position and orientation as target
      </button>

      <button onClick={handleResetStateClick} id='reset-state-button'>
        Reset state
      </button>
    </>
  )
}

const TabButton = styled.button<{
  selected: boolean
}>`
  all: unset;
  background-color: ${(p) => (p.selected ? p.theme.colors.lightergray : p.theme.colors.lightgray)};
  padding: 0.3rem;
  border: 1px solid ${(p) => p.theme.colors.gray};
`

export default Targets
