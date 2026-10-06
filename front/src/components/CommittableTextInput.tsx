import React, { useState, useEffect } from 'react'
import styled from 'styled-components'

import { palette } from 'styles/General'

const StyledInput = styled.input<{ width?: string }>`
  width: ${props => props.width || '130px'};
  height: 24px;
  box-sizing: border-box;
  padding: 0 8px;
  border: 1px solid ${palette.borderStrong};
  border-radius: 3px;
  outline: none;
  transition: border-color 0.15s;
  background-color: ${palette.surfaceRaised};
  color: ${palette.text};
  font-size: 12px;
  font-family: inherit;
  margin-right: 17px;

  &:focus {
    border-color: ${palette.accent};
  }

  &:disabled {
    background-color: ${palette.surface};
    border-color: ${palette.border};
    color: ${palette.textMuted};
  }
`

const StyledTextarea = styled.textarea<{ width?: string }>`
  width: ${props => props.width || '315px'};
  height: 58px;
  box-sizing: border-box;
  flex-shrink: 0;
  padding: 4px 8px;
  line-height: 16px;
  border: 1px solid ${palette.borderStrong};
  border-radius: 3px;
  outline: none;
  transition: border-color 0.15s;
  background-color: ${palette.surfaceRaised};
  color: ${palette.text};
  font-size: 12px;
  resize: none;
  font-family: inherit;
  margin-right: 17px;

  &:focus {
    border-color: ${palette.accent};
  }

  &:disabled {
    background-color: ${palette.surface};
    border-color: ${palette.border};
    color: ${palette.textMuted};
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