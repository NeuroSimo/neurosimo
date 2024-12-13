import React, { ReactNode, useEffect, useState } from 'react'
import styled from 'styled-components'

import { getSequenceIndexFromRowId, useFocusMemo } from 'utils'
import Rectangle from '../../Rectangle'
import { CellProps } from 'types/table'
import { useAppSelector } from 'providers/reduxHooks'
import { updateTargetInRos } from 'ros/services/target'
import { updatePulseSequenceInRos } from 'ros/services/pulseSequence'
import { updatePulseInRos } from 'ros/services/pulse'

interface EditableCellProps extends CellProps {
  expandElement?: ReactNode
  whiteSpace?: boolean
}

interface UpdateEditableCellProps extends EditableCellProps {
  updateData: (rowIndex: number, columnName: string, value: any, toggle: boolean) => void
}

export const EditableSequenceTableCell = (props: EditableCellProps) => {
  const { row } = props

  const isTarget = props.row.depth > 0
  const { targets } = useAppSelector((state) => state.targets)
  const { sequences } = useAppSelector((state) => state.sequences)

  const sequenceIndex = getSequenceIndexFromRowId(row.id)

  const updateTargetData = (rowIndex: number, columnName: string, value: any, toggle: boolean) => {
    const key = columnName.slice(3).toLowerCase()

    //update only name for target (so changes are reflected in all pulses with this target in all sequences),
    //otherwise update the pulse
    if (key === 'name') {
      const targetIndex = sequences[sequenceIndex].pulses[rowIndex].targetIndex
      const target = targets[targetIndex]
      updateTargetInRos(target, key, value, toggle)
    } else {
      const sequenceId = getSequenceIndexFromRowId(row.id)
      updatePulseInRos(sequences[sequenceId], rowIndex, key, value, toggle)
    }
  }

  const updateSequenceData = (rowIndex: number, columnName: string, value: any, toggle: boolean) => {
    const key = columnName.slice(3).toLowerCase()
    const sequence = sequences[rowIndex]

    updatePulseSequenceInRos(sequence, key, value, toggle)
  }

  return <EditableCell {...props} updateData={isTarget ? updateTargetData : updateSequenceData} />
}

export const EditableTargetTableCell = (props: UpdateEditableCellProps) => {
  const { targets } = useAppSelector((state) => state.targets)

  const updateTargetData = (rowIndex: number, columnName: string, value: any, toggle: boolean) => {
    const target = targets[rowIndex]
    updateTargetInRos(target, columnName, value, toggle)
  }

  return <EditableCell {...props} updateData={updateTargetData} />
}

const EditableCell = ({
  value: initialValue,
  row,
  column,
  updateData,
  expandElement,
  whiteSpace,
}: UpdateEditableCellProps) => {
  // We need to keep and update the state of the cell normally
  const [value, setValue] = useState(initialValue)
  const [changed, setChanged] = useState<boolean>(false)
  const [beforeChange, setBeforeChange] = useState(initialValue)
  const [toggle, setToggle] = useState<boolean>(true)
  const valueType = typeof initialValue

  const { index, depth, id } = row

  const [inputRef, setInputFocus] = useFocusMemo()

  const onChange = (e: any) => {
    const newValue = valueType === 'number' ? Number(e.target.value) : e.target.value
    setValue(newValue)
    setChanged(true)
  }

  // We'll only update the external data when the input is blurred
  const onBlur = () => {
    if (value && value.length === 0) {
      setValue(beforeChange)
      // do not update data if value is not changed
      return
    } else {
      setBeforeChange(value)
    }

    if (changed && column.id) {
      updateData(index, column.id, value, false)
      setChanged(false)
    }
    setToggle(true)
  }

  const handleKeyPress = (event: any) => {
    if (event.key === 'Enter') {
      inputRef.current?.blur()
    }
  }

  // If the initialValue is changed external, sync it up with our state
  useEffect(() => {
    setValue(initialValue)
  }, [initialValue])

  const onDoubleClick = () => {
    setToggle(false)

    // Needs to be in setTimeout so the input element can render, otherwise it would be impossible to setInputFocus
    // as the element does not exist
    setTimeout(() => {
      setInputFocus()
    }, 100)
  }

  return (
    <ContainerWithMargin id={`cell-container-${id}-${column.id}`}>
      {whiteSpace ? (
        <>
          <WhiteSpace cols={depth} />
          <Rectangle color={'red'} />
        </>
      ) : null}
      {toggle ? (
        <Container onDoubleClick={onDoubleClick} className={'cell-value-container'}>
          <span>{value}</span>
          {expandElement}
        </Container>
      ) : (
        <CellInput value={value} onChange={onChange} onBlur={onBlur} ref={inputRef} onKeyPress={handleKeyPress} />
      )}
    </ContainerWithMargin>
  )
}

const ContainerWithMargin = styled.span`
  display: flex;
`

const WhiteSpace = styled.span<{
  cols: number
}>`
  background-color: rgba(255, 255, 255, 0);
  width: ${(p) => p.cols * 20}px;
  height: 100%;
  overflow: hidden;
  writing-mode: horizontal-tb;
  border: 0;
`
const Container = styled.span`
  display: flex;
  width: 100%;
  border: 0;
  background-color: inherit;
  font-size: 1rem;
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Roboto', 'Oxygen', 'Ubuntu', 'Cantarell', 'Fira Sans',
    'Droid Sans', 'Helvetica Neue', sans-serif;
  overflow: auto;
  overflow-y: hidden;
  min-height: 1rem;
`

const CellInput = styled.input`
  all: unset;
  width: 100%;
  border: 0;
  background-color: inherit;
  font-size: 1rem;
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Roboto', 'Oxygen', 'Ubuntu', 'Cantarell', 'Fira Sans',
    'Droid Sans', 'Helvetica Neue', sans-serif;
`
