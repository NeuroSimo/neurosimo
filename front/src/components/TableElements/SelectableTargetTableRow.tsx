import React, { useEffect, useRef, useState } from 'react'
import styled from 'styled-components'
import { DragSourceMonitor, useDrag, useDrop } from 'react-dnd'

import { useAppSelector } from 'providers/reduxHooks'

import Dots from '../Dots'
import { changeTargetIndexInRos, updateTargetInRos } from 'ros/services/target'

const SelectableTargetTableRow = (props: any) => {
  const { index } = props
  const { targets } = useAppSelector((state) => state.targets)
  const [selected, setSelected] = useState(targets[index].selected)

  const dropRef = useRef<HTMLTableRowElement>(null)
  const dragRef = useRef<HTMLTableCellElement>(null)

  const moveRow = (dragIndex: number, hoverIndex: number) => {
    const target = targets[dragIndex]
    changeTargetIndexInRos(target, hoverIndex)
  }

  const [, drop] = useDrop({
    accept: 'row',
    hover(item: any, monitor) {
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

  useEffect(() => {
    setSelected(targets[index].selected)
  }, [targets[index].selected])

  const onClick = (event: any) => {
    event.preventDefault()

    const target = targets[index]
    updateTargetInRos(target, 'selected', !selected, true)
  }

  const [{ isDragging }, drag, preview] = useDrag({
    item: { type: 'row', index },
    type: 'row',
    collect: (monitor: DragSourceMonitor) => ({
      isDragging: monitor.isDragging(),
    }),
  })

  const opacity = isDragging ? 0 : 1

  preview(drop(dropRef))
  drag(dragRef)

  return (
    <TableRow id={`target-${index}`} {...props} selected={selected} opacity={opacity} onClick={onClick} ref={dropRef}>
      <td ref={dragRef}>
        <Dots />
      </td>
      {props.children}
    </TableRow>
  )
}

const TableRow = styled.tr<{
  selected: boolean
  opacity: number
}>`
  border-bottom: 1px solid ${(p) => p.theme.colors.gray};
  background-color: ${(p) => (p.selected ? p.theme.colors.lightgray : p.theme.colors.white)};
  opacity: ${(p) => p.opacity};
`

export default SelectableTargetTableRow
