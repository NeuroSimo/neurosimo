import React, { useState, useEffect } from 'react'
import styled from 'styled-components'

import { palette } from 'styles/General'

const Container = styled.div`
  display: flex;
  align-items: center;
  width: 150px;
  justify-content: flex-end;
  margin-right: 17px;
`

const PrefixLabel = styled.span<{ disabled?: boolean }>`
  font-size: 11px;
  font-family: ${palette.monoFont};
  color: ${props => props.disabled ? palette.textDim : palette.textMuted};
  margin-right: 4px;
  user-select: none;
`

const NumericInput = styled.input<{ width?: string; disabled?: boolean }>`
  width: ${props => props.width || '45px'};
  height: 22px;
  box-sizing: content-box;
  padding: 0 6px;
  border: 1px solid ${palette.borderStrong};
  border-radius: 3px;
  outline: none;
  transition: border-color 0.15s;
  background-color: ${palette.surfaceRaised};
  color: ${palette.text};
  font-size: 11.5px;
  font-family: ${palette.monoFont};

  &:focus {
    border-color: ${palette.accent};
  }

  /* Same faded look as a disabled Select. */
  &:disabled {
    background-color: ${palette.surface};
    border-color: ${palette.border};
    color: ${palette.textMuted};
    opacity: 0.7;
    cursor: not-allowed;
  }
`

interface CommittableNumericInputProps {
  value: number
  onCommit: (value: number) => void
  prefix: string
  maxLength: number
  min?: number
  placeholder?: string
  disabled?: boolean
  width?: string
}

export const CommittableNumericInput: React.FC<CommittableNumericInputProps> = ({
  value,
  onCommit,
  prefix,
  maxLength,
  min,
  placeholder,
  disabled = false,
  width,
}) => {
  const formatNumericPart = (numericValue: number) => {
    if (!numericValue || numericValue <= 0) {
      return ''
    }
    return String(numericValue).padStart(maxLength, '0')
  }

  const [inputValue, setInputValue] = useState(formatNumericPart(value))

  useEffect(() => {
    setInputValue(formatNumericPart(value))
  }, [value, maxLength])

  const handleChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const newValue = event.target.value
    const filteredValue = newValue.replace(/\D/g, '').substring(0, maxLength)
    setInputValue(filteredValue)
  }

  const handleCommit = () => {
    const paddedValue = inputValue.padStart(maxLength, '0')
    let numericValue = parseInt(paddedValue, 10)
    if (min !== undefined && numericValue < min) {
      numericValue = min
    }
    onCommit(numericValue)
    setInputValue(String(numericValue).padStart(maxLength, '0'))
  }

  const handleKeyDown = (event: React.KeyboardEvent<HTMLInputElement>) => {
    if (event.key === 'Enter') {
      handleCommit()
      event.currentTarget.blur()
    }
  }

  const handleBlur = () => {
    handleCommit()
  }

  return (
    <Container>
      <PrefixLabel disabled={disabled}>{prefix}</PrefixLabel>
      <NumericInput
        type="text"
        value={inputValue}
        onChange={handleChange}
        onKeyDown={handleKeyDown}
        onBlur={handleBlur}
        placeholder={placeholder}
        disabled={disabled}
        width={width}
      />
    </Container>
  )
}
