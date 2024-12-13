import React, { useEffect, useState } from 'react'
import Expand from '../../Expand'
import { EditableSequenceTableCell } from './EditableCell'
import { CellProps } from 'types/table'
import { useAppDispatch } from 'providers/reduxHooks'
import { setExpandedSequence } from 'reducers/sequenceReducer'

const ExpandableCell = ({ value, row, column }: CellProps) => {
  const { canExpand, depth } = row
  const dispatch = useAppDispatch()

  const [isExpanded, setIsExpanded] = useState(row.isExpanded)

  useEffect(() => {
    setIsExpanded(row.isExpanded)
  }, [row.isExpanded])

  const handleClickExpand = (event: any) => {
    event.stopPropagation()
    row.toggleRowExpanded()

    dispatch(
      setExpandedSequence({
        index: row.index,
        expanded: !isExpanded,
      }),
    )
  }
  const expandElement = canExpand ? <Expand onClick={handleClickExpand} expanded={isExpanded} /> : null

  return (
    <EditableSequenceTableCell
      value={value}
      row={row}
      column={column}
      expandElement={expandElement}
      whiteSpace={depth > 0}
    />
  )
}

export default ExpandableCell
