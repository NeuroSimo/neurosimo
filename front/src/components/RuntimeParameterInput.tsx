import React, { useState, useEffect } from 'react'
import styled from 'styled-components'

import { RuntimeParameterInfo } from 'ros/experiment'
import { RuntimeParameterValue } from 'providers/SessionConfigProvider'

const TextInput = styled.input<{ disabled?: boolean }>`
  width: 90px;
  padding: 5px;
  border: 1px solid #ccc;
  border-radius: 3px;
  outline: none;
  transition: background-color 0.2s;
  font-size: 11px;
  font-family: 'Courier New', monospace;

  &:focus {
    background-color: #f0f8ff;
  }

  &:disabled {
    background-color: #f5f5f5;
    color: #ccc;
    cursor: not-allowed;
  }
`

const Unit = styled.span<{ disabled?: boolean }>`
  font-size: 11px;
  color: ${props => (props.disabled ? '#ccc' : '#666')};
  margin-left: 4px;
  user-select: none;
`

interface RuntimeParameterInputProps {
  descriptor: RuntimeParameterInfo
  value: RuntimeParameterValue | undefined
  onCommit: (value: RuntimeParameterValue) => void
  disabled?: boolean
}

const clampNumber = (value: number, descriptor: RuntimeParameterInfo): number => {
  let result = value
  if (descriptor.has_min && result < descriptor.min) {
    result = descriptor.min
  }
  if (descriptor.has_max && result > descriptor.max) {
    result = descriptor.max
  }
  return result
}

export const RuntimeParameterInput: React.FC<RuntimeParameterInputProps> = ({
  descriptor,
  value,
  onCommit,
  disabled = false,
}) => {
  const isNumeric = descriptor.type === 'float' || descriptor.type === 'int'

  /* Boolean parameters render as a checkbox and commit immediately. */
  if (descriptor.type === 'bool') {
    return (
      <input
        type="checkbox"
        checked={value === true}
        disabled={disabled}
        onChange={(event) => onCommit(event.target.checked)}
      />
    )
  }

  const [inputValue, setInputValue] = useState(value === undefined ? '' : String(value))

  useEffect(() => {
    setInputValue(value === undefined ? '' : String(value))
  }, [value])

  const handleCommit = () => {
    if (!isNumeric) {
      onCommit(inputValue)
      return
    }

    if (inputValue.trim() === '') {
      /* Leave the field empty (value cleared) so required validation can flag it. */
      return
    }

    const parsed = descriptor.type === 'int' ? parseInt(inputValue, 10) : parseFloat(inputValue)
    if (Number.isNaN(parsed)) {
      /* Restore the previous value on invalid input. */
      setInputValue(value === undefined ? '' : String(value))
      return
    }

    const clamped = clampNumber(parsed, descriptor)
    onCommit(clamped)
    setInputValue(String(clamped))
  }

  const handleKeyDown = (event: React.KeyboardEvent<HTMLInputElement>) => {
    if (event.key === 'Enter') {
      handleCommit()
      event.currentTarget.blur()
    }
  }

  return (
    <>
      <TextInput
        type="text"
        value={inputValue}
        onChange={(event) => setInputValue(event.target.value)}
        onKeyDown={handleKeyDown}
        onBlur={handleCommit}
        placeholder="required"
        disabled={disabled}
      />
      {descriptor.unit && <Unit disabled={disabled}>{descriptor.unit}</Unit>}
    </>
  )
}
