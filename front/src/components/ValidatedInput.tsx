import React, { useState, useEffect } from 'react'
import styled from 'styled-components'

const StyledInput = styled.input<{ value: string }>`
  position: relative;
  display: inline-block;
  width: 50px;
  height: 20px;
  padding: 4px 8px;
  text-align: right;
  border: 1px solid #ccc;
  border-radius: 4px;
  background-color: transparent;
  color: #000;
  font-size: 14px;

  /* the “track” behind the text */
  &::before {
    content: '';
    position: absolute;
    top: 50%;
    left: 0;
    height: 6px;
    border-radius: 3px;
    background: #333;
    width: ${(p) => {
      const num = parseFloat(p.value) || 0
      const pct = Math.min(Math.max((num / 7200) * 100, 0), 100)
      return `${pct}%`
    }};
    transform: translateY(-50%);
    z-index: -1;
  }

  /* hide the native spinners */
  &::-webkit-inner-spin-button,
  &::-webkit-outer-spin-button {
    -webkit-appearance: none;
    margin: 0;
  }
`

interface ValidatedInputProps {
  value: number
  onChange: (newValue: number) => void
  type?: string
  min?: number
  max?: number
  step?: number
  disabled?: boolean
}

export const ValidatedInput: React.FC<ValidatedInputProps> = ({
  value,
  onChange,
  type = 'number',
  min,
  max,
  step,
  disabled,
}) => {
  const [localValue, setLocalValue] = useState<string>(value.toString())

  const handleChange = (inputValue: string) => {
    setLocalValue(inputValue)
    const num = parseFloat(inputValue)
    if (!isNaN(num) && (min === undefined || num >= min) && (max === undefined || num <= max)) {
      onChange(num)
    }
  }

  const handleBlur = () => {
    const num = parseFloat(localValue)
    if (isNaN(num)) {
      // revert if not a number
      setLocalValue(value.toString())
      return
    }

    // clamp to [min, max]
    let clamped = num
    if (min !== undefined && num < min) clamped = min
    if (max !== undefined && num > max) clamped = max

    // round to one decimal
    clamped = Math.round(clamped * 10) / 10

    if (clamped !== value) {
      onChange(clamped)
    }
    setLocalValue(clamped.toString())
  }

  useEffect(() => {
    setLocalValue(value.toString())
  }, [value])

  return (
    <StyledInput
      type={type}
      value={localValue}
      min={min}
      max={max}
      step={step}
      disabled={disabled}
      onChange={e => handleChange(e.target.value)}
      onBlur={handleBlur}
    />
  )
}
