import React, { useState, useEffect } from 'react'
import styled from 'styled-components'

import { palette } from 'styles/General'

const StyledInput = styled.input<{ valid?: boolean; width?: string }>`
  height: 22px;
  box-sizing: content-box;
  padding: 0 8px;
  border: 1px solid ${(props) => (props.valid ? palette.borderStrong : palette.red)};
  border-radius: 3px;
  font-size: 11.5px;
  font-family: ${palette.monoFont};
  background-color: ${palette.surfaceRaised};
  color: ${palette.text};
  width: ${(props) => props.width || 'auto'};
  
  &:focus {
    outline: none;
    border-color: ${(props) => (props.valid ? palette.accent : palette.red)};
  }

  &:disabled {
    background-color: ${palette.surface};
    border-color: ${palette.border};
    color: ${palette.textMuted};
    cursor: not-allowed;
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
    const parsedValue = parse(localValue)
    if (isNaN(parsedValue)) {
      setLocalValue(format(value))
      return
    }

    // Clip the value to min/max bounds
    let clippedValue = parsedValue
    if (min !== undefined && parsedValue < min) {
      clippedValue = min
    } else if (max !== undefined && parsedValue > max) {
      clippedValue = max
    }

    onChange(clippedValue)
    setLocalValue(format(clippedValue))
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
