import React, { useState, useEffect } from 'react'
import styled from 'styled-components'

const StyledInput = styled.input<{ width?: string }>`
  width: ${props => props.width || '130px'};
  padding: 5px;
  border: 1px solid #ccc;
  border-radius: 3px;
  outline: none;
  transition: background-color 0.2s;
  font-size: 11px;
  margin-right: 17px;

  &:focus {
    background-color: #f0f8ff;
  }
`

const StyledTextarea = styled.textarea<{ width?: string }>`
  width: ${props => props.width || '315px'};
  height: 60px;
  padding: 5px;
  border: 1px solid #ccc;
  border-radius: 3px;
  outline: none;
  transition: background-color 0.2s;
  font-size: 11px;
  resize: none;
  font-family: inherit;
  margin-right: 17px;

  &:focus {
    background-color: #f0f8ff;
  }
`

interface CommittableTextInputProps {
  value: string
  onCommit: (value: string) => void
  placeholder?: string
  disabled?: boolean
  multiline?: boolean
  width?: string
}

export const CommittableTextInput: React.FC<CommittableTextInputProps> = ({
  value,
  onCommit,
  placeholder,
  disabled = false,
  multiline = false,
  width,
}) => {
  const [localValue, setLocalValue] = useState(value)

  // Sync local state with external value when it changes
  useEffect(() => {
    setLocalValue(value)
  }, [value])

  const commitValue = () => {
    if (localValue !== value) {
      onCommit(localValue)
    }
  }

  const handleChange = (event: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement>) => {
    setLocalValue(event.target.value)
  }

  const handleKeyDown = (event: React.KeyboardEvent<HTMLInputElement | HTMLTextAreaElement>) => {
    if (event.key === 'Enter' && (!multiline || !event.shiftKey)) {
      commitValue()
      event.currentTarget.blur()
    }
  }

  const handleBlur = () => {
    commitValue()
  }

  if (multiline) {
    return (
      <StyledTextarea
        value={localValue}
        onChange={handleChange}
        onKeyDown={handleKeyDown}
        onBlur={handleBlur}
        disabled={disabled}
        placeholder={placeholder}
        width={width}
      />
    )
  }

  return (
    <StyledInput
      type="text"
      value={localValue}
      onChange={handleChange}
      onKeyDown={handleKeyDown}
      onBlur={handleBlur}
      disabled={disabled}
      placeholder={placeholder}
      width={width}
    />
  )
}