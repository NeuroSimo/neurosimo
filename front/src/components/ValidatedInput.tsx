import React, { useState, useEffect } from 'react'
import styled from 'styled-components'

const StyledInput = styled.input<{ valid?: boolean; width?: string }>`
  padding: 8px 12px;
  border: 1px solid ${(props) => (props.valid ? '#ddd' : 'red')};
  border-radius: 4px;
  font-size: 14px;
  background-color: white;
  color: black;
  width: ${(props) => props.width || 'auto'};
  
  &:focus {
    outline: none;
    border-color: ${(props) => (props.valid ? '#007bff' : 'red')};
    box-shadow: 0 0 0 2px ${(props) => (props.valid ? 'rgba(0, 123, 255, 0.25)' : 'rgba(255, 0, 0, 0.25)')};
  }
`

interface ValidatedInputProps {
  value: number
  onChange: (newValue: number) => void
  formatValue?: (value: number) => string
  parseValue?: (value: string) => number
  type?: string
  min?: number
  max?: number
  step?: number
  disabled?: boolean
  width?: string
}

export const ValidatedInput: React.FC<ValidatedInputProps> = ({
  value,
  onChange,
  formatValue,
  parseValue,
  type = 'number',
  min,
  max,
  width,
  ...props
}) => {
  const defaultFormat = (val: number) => val.toString()
  const defaultParse = (val: string) => parseFloat(val)
  
  const format = formatValue || defaultFormat
  const parse = parseValue || defaultParse
  
  const [localValue, setLocalValue] = useState<string>(format(value))

  const isValueValid = (strValue: string): boolean => {
    const numValue = parse(strValue)
    if (isNaN(numValue)) return false
    return (min === undefined || numValue >= min) && (max === undefined || numValue <= max)
  }

  const handleChange = (inputValue: string) => {
    setLocalValue(inputValue)
  }

  const handleBlur = () => {
    if (!isValueValid(localValue)) {
      setLocalValue(format(value))
    } else {
      const parsedValue = parse(localValue)
      onChange(parsedValue)
      setLocalValue(format(parsedValue))
    }
  }

  useEffect(() => {
    setLocalValue(format(value))
  }, [value])

  return (
    <StyledInput
      type={type}
      {...props}
      value={localValue}
      valid={isValueValid(localValue)}
      width={width}
      onChange={(e) => handleChange(e.target.value)}
      onBlur={handleBlur}
    />
  )
}
