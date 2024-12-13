import React, { ReactNode, useEffect, useRef, useState } from 'react'
import styled from 'styled-components'

import { useAppSelector } from 'providers/reduxHooks'

import { getSequenceIndexFromRowId } from 'utils'
import { DragSourceMonitor, useDrag, useDrop } from 'react-dnd'
import Dots from '../Dots'
import { updatePulseSequenceInRos } from 'ros/services/pulseSequence'
import { updatePulseIndexInRos, updatePulseInRos } from 'ros/services/pulse'

interface Props {
  index: number
  isTarget: boolean
  rowId: string
  children: ReactNode
}

interface DragItem {
  type: 'row'
  index: number
  sequenceIndex: number
}

const SelectableSequenceTableRow = (props: Props) => {
  const { index, isTarget, rowId } = props

  const { sequences } = useAppSelector((state) => state.sequences)

  const sequenceIndex = isTarget ? getSequenceIndexFromRowId(rowId) : index

  const sequence = sequences[sequenceIndex]
  const [selected, setSelected] = useState(isTarget ? sequence.pulses[index]?.selected : sequence.selected)

  const dropRef = useRef<HTMLTableRowElement>(null)
  const dragRef = useRef<HTMLTableCellElement>(null)

  useEffect(() => {
    if (isTarget) {
      setSelected(sequences[sequenceIndex].pulses[index].selected)
    } else {
      setSelected(sequence.selected)
    }
  }, [sequences])

  const moveRow = (dragIndex: number, hoverIndex: number) => {
    updatePulseIndexInRos(sequence, dragIndex, hoverIndex)
  }

  const [, drop] = useDrop({
    accept: 'row',
    hover: (item: DragItem, monitor) => {
      //disable moving targets across sequences
      if (item.sequenceIndex !== sequenceIndex) return

      if (!dropRef.current) {
        return
      }
      const dragIndex: number = item.index
      const hoverIndex: number = index
      // Don't replace items with themselves
      if (dragIndex === hoverIndex) {
        return
      }
      // Determine rectangle on screen
      const hoverBoundingRect = dropRef.current.getBoundingClientRect()
      // Get vertical middle
      const hoverMiddleY = (hoverBoundingRect.bottom - hoverBoundingRect.top) / 2

      // Determine mouse position
      const clientOffset = monitor.getClientOffset()
      if (!clientOffset || !clientOffset.y) {
        return
      }

      // Get pixels to the top
      const hoverClientY = clientOffset.y - hoverBoundingRect.top

      // Only perform the move when the mouse has crossed half of the items height
      // When dragging downwards, only move when the cursor is below 50%
      // When dragging upwards, only move when the cursor is above 50%
      // Dragging downwards
      if (dragIndex < hoverIndex && hoverClientY < hoverMiddleY) {
        return
      }
      // Dragging upwards
      if (dragIndex > hoverIndex && hoverClientY > hoverMiddleY) {
        return
      }
      // Time to actually perform the action
      moveRow(dragIndex, hoverIndex)
      // Note: we're mutating the monitor item here!
      // Generally it's better to avoid mutations,
      // but it's good here for the sake of performance
      // to avoid expensive index searches.
      item.index = hoverIndex
    },
  })

  const onClick = (event: any) => {
    event.preventDefault()

    if (isTarget) {
      updatePulseInRos(sequence, index, 'selected', !selected, true)
    } else {
      updatePulseSequenceInRos(sequence, 'selected', !selected, true)
    }
  }

  const [{ isDragging }, drag, preview] = useDrag({
    item: {
      type: 'row',
      index,
      sequenceIndex,
    },
    type: 'row',
    collect: (monitor: DragSourceMonitor) => ({
      isDragging: monitor.isDragging(),
    }),
  })

  preview(drop(dropRef))
  drag(dragRef)

  return (
    <TableRow {...props} selected={selected} onClick={onClick} ref={dropRef}>
      {isTarget ? (
        <td ref={dragRef}>
          <Dots />
        </td>
      ) : (
        <td></td>
      )}

      {props.children}
    </TableRow>
  )
}

const TableRow = styled.tr<{
  selected: boolean
}>`
  border-bottom: 1px solid ${(p) => p.theme.colors.gray};
  background-color: ${(p) => (p.selected ? p.theme.colors.lightgray : p.theme.colors.white)};
`

export default SelectableSequenceTableRow
