import React, { useState, useEffect } from 'react'
import styled from 'styled-components'

const Container = styled.div`
  display: flex;
  align-items: center;
  width: 150px;
  justify-content: flex-end;
  margin-right: 17px;
`

const PrefixLabel = styled.span<{ disabled?: boolean }>`
  font-size: 11px;
  color: ${props => props.disabled ? '#ccc' : '#666'};
  margin-right: 4px;
  user-select: none;
`

const NumericInput = styled.input<{ width?: string; disabled?: boolean }>`
  width: ${props => props.width || '45px'};
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
