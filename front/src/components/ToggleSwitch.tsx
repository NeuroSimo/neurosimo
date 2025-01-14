import React from 'react'

import './ToggleSwitch.scss'

type ToggleProps = {
  type: 'light' | 'ios' | 'skewed' | 'flat' | 'flip'
  onLabel?: string
  offLabel?: string
  checked?: boolean
  disabled?: boolean
  onChange?: (checked: boolean) => void
}

let idCounter = 0

export const ToggleSwitch: React.FC<ToggleProps> = ({
  type,
  onLabel = '',
  offLabel = '',
  checked = false,
  disabled = false,
  onChange,
}) => {
  const inputId = `toggle-${type}-${idCounter}`
  idCounter++

  return (
    <div className='tg-list-item'>
      <input
        className={`tgl tgl-${type}`}
        id={inputId}
        type='checkbox'
        checked={checked}
        disabled={disabled}
        onChange={(e) => onChange && onChange(e.target.checked)}
      />
      <label className='tgl-btn' data-tg-off={offLabel} data-tg-on={onLabel} htmlFor={inputId}></label>
    </div>
  )
}
