import React, { useMemo } from 'react'

import { MenuItem, SubMenu } from '@szhsin/react-menu'
import '@szhsin/react-menu/dist/index.css'

import { useAppSelector } from 'providers/reduxHooks'

import { Pulse, PulseSequence } from '../types/pulseSequence'
import Eye from './Eye'
import { EyeCell } from './TableElements/Cells/EyeCell'
import { EditableTargetTableCell } from './TableElements/Cells/EditableCell'
import { GenericTable } from './GenericTable'
import SelectableTargetTableRow from './TableElements/SelectableTargetTableRow'
import { createPulsesFromSelectedTargets } from 'utils'
import { addPulseSequenceToRos, addPulseToPulseSequenceInRos } from 'ros/services/pulseSequence'
import { openTargetOrientationDialogInNeuronavigation, removeTargetInRos } from 'ros/services/target'

const TargetTable = () => {
  const { sequences } = useAppSelector((state) => state.sequences)
  const { targets } = useAppSelector((state) => state.targets)

  const columns = useMemo(
    () => [
      {
        Header: () => <Eye visible={true} />,
        accessor: 'visible',
        width: 40,
        Cell: EyeCell,
      },
      {
        Header: 'Name',
        accessor: 'name',
        width: 'auto',
        Cell: EditableTargetTableCell,
      },
      {
        Header: 'Type',
        accessor: 'type',
        width: 'auto',
      },
      {
        Header: 'Comment',
        accessor: 'comment',
        width: 'auto',
        Cell: EditableTargetTableCell,
      },
    ],
    [],
  )

  const handleNewSequence = (event: any) => {
    const pulses: Pulse[] = createPulsesFromSelectedTargets(targets)
    if (pulses.length === 0) {
      console.log('no targets selected for new sequence')
      return
    }

    addPulseSequenceToRos(pulses)
  }
  const filterTargetKeys = () => {
    return targets.map((target) => {
      return {
        name: target.name,
        comment: target.comment,
        type: target.type,
        visible: target.visible,
        selected: target.selected,
      }
    })
  }
  const handleAddToSequence = (sequence: PulseSequence) => {
    const selectedPulses = createPulsesFromSelectedTargets(targets)
    if (selectedPulses.length === 0) {
      console.log('no targets selected for new sequence')
      return
    }

    selectedPulses.forEach((pulse) => {
      addPulseToPulseSequenceInRos(sequence, pulse)
    })
  }

  const handleRemoveTargets = () => {
    targets
      .filter((target) => target.selected)
      .forEach((target) => {
        removeTargetInRos(target)
      })
  }

  const handleOpenOrientationDialog = () => {
    targets
      .filter((target) => target.selected)
      .forEach((target) => {
        openTargetOrientationDialogInNeuronavigation(targets.indexOf(target))
      })
  }

  const createMenu = () => {
    return (
      <>
        <MenuItem onClick={handleNewSequence} id='create-new-sequence'>
          New sequence from selection
        </MenuItem>
        <MenuItem onClick={handleRemoveTargets}>Remove selected target(s)</MenuItem>
        <SubMenu label='Add to sequence' id={'add-to-sequence'}>
          {sequences.map((seq, i) => {
            return (
              <MenuItem onClick={() => handleAddToSequence(seq)} key={seq.name} id={`add-to-sequence-${i}`}>
                {seq.name}
              </MenuItem>
            )
          })}
        </SubMenu>
        <MenuItem onClick={handleOpenOrientationDialog}>Open orientation dialog for target(s)</MenuItem>
      </>
    )
  }

  return (
    <GenericTable
      columns={columns}
      data={filterTargetKeys()}
      createMenu={createMenu}
      SelectableRow={SelectableTargetTableRow}
    />
  )
}

export default TargetTable
