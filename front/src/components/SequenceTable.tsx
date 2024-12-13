import React, { useMemo } from 'react'

import { MenuItem } from '@szhsin/react-menu'
import '@szhsin/react-menu/dist/index.css'

import Eye from './Eye'
import { EyeCell } from './TableElements/Cells/EyeCell'
import { EditableSequenceTableCell } from './TableElements/Cells/EditableCell'
import { GenericTable } from './GenericTable'
import ExpandableCell from './TableElements/Cells/ExpandableCell'
import SelectableSequenceTableRow from './TableElements/SelectableSequenceTableRow'
import { useAppSelector } from 'providers/reduxHooks'
import { PulseSequence } from 'types/pulseSequence'
import styled from 'styled-components'
import { removePulseInRos } from 'ros/services/pulse'
import { addPulseToPulseSequenceInRos, removePulseSequenceInRos } from 'ros/services/pulseSequence'

const SequenceTable = () => {
  const { sequences } = useAppSelector((state) => state.sequences)
  const { targets } = useAppSelector((state) => state.targets)

  const columns = useMemo(
    () => [
      {
        Header: () => <Eye visible={true} />,
        accessor: 'seqVisible',
        width: 40,
        Cell: EyeCell,
      },
      {
        Header: 'Name',
        accessor: 'seqName',
        width: 'auto',
        Cell: ExpandableCell,
      },
      {
        Header: 'Intensity',
        accessor: 'seqIntensity',
        width: 'auto',
        Cell: EditableSequenceTableCell,
      },
      {
        Header: 'isi',
        accessor: 'seqIsi',
        width: 'auto',
        Cell: EditableSequenceTableCell,
      },
      {
        Header: 'Mode Duration',
        accessor: 'seqModeDuration',
        width: 'auto',
        Cell: EditableSequenceTableCell,
      },
    ],
    [],
  )
  const filterSequenceKeys = () => {
    return sequences.map((seq) => {
      const subRows = seq.pulses.map((pulse) => {
        const target = targets[pulse.targetIndex]

        return {
          seqName: target.name,
          seqComment: target.comment,
          seqIntensity: pulse.intensity,
          seqVisible: pulse.visible,
          selected: pulse.selected,
          seqIsi: pulse.isi,
        }
      })

      return {
        seqName: seq.name,
        seqComment: seq.comment,
        seqIntensity: seq.intensity,
        seqVisible: seq.visible,
        selected: seq.selected,
        subRows,
        seqIsi: seq.isi,
      }
    })
  }

  const handleDuplicateTargets = () => {
    sequences.forEach((seq: PulseSequence, index: number) => {
      const selectedPulses = seq.pulses.filter((pulse) => pulse.selected)

      selectedPulses.forEach((pulse) => {
        addPulseToPulseSequenceInRos(seq, pulse)
      })
    })
  }

  const handleRemovePulses = () => {
    sequences.forEach((seq: PulseSequence) => {
      seq.pulses
        .filter((pulse) => pulse.selected)
        .forEach((pulse) => {
          const pulseIndex = seq.pulses.indexOf(pulse)
          removePulseInRos(seq, pulseIndex)
        })
    })
  }

  const handleRemovePulseSequences = () => {
    sequences
      .filter((seq) => seq.selected)
      .forEach((seq) => {
        removePulseSequenceInRos(seq)
      })
  }

  const createMenu = () => {
    return (
      <>
        <MenuItem onClick={handleDuplicateTargets}>Duplicate selected target(s)</MenuItem>
        <MenuItem onClick={handleRemovePulses}>Remove selected target(s)</MenuItem>
        <MenuItem onClick={handleRemovePulseSequences}>Remove selected pulse sequences(s)</MenuItem>
      </>
    )
  }

  return (
    <GenericTable
      columns={columns}
      data={filterSequenceKeys()}
      createMenu={createMenu}
      SelectableRow={SelectableSequenceTableRow}
    />
  )
}

const Wrapper = styled.div`
  all: unset;
`

export default SequenceTable
