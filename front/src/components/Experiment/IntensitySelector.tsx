import React, { useState } from 'react'
import styled from 'styled-components'

import { SmallerTitle } from 'styles/ExperimentStyles'
import { ValidatedInput } from 'components/ValidatedInput'

interface IntensitySelectorProps {
  value: number
  min: number
  max: number
  showMaximumIntensity: boolean
  maximumIntensity: number
  onValueChange: (value: number) => void
}

const MaximumIntensityLabel = styled.span`
  position: absolute;
  font-size: 12px; /* Adjust as needed */
  color: red;
  right: 80%;
  transform: translateY(-50%);
`

const MaximumIntensityLine = styled.div<{ top: string }>`
  position: absolute;
  width: 25%;
  height: 2px;
  background-color: red;
  top: ${(props) => props.top};
  right: 50%;
  z-index: 1;
`

const IntensitySelectorContainer = styled.div`
  display: flex;
  flex-direction: column;
  align-items: center;
  position: relative;
  width: 90px;
`

const IntensityLabel = styled.span`
  font-size: 0.85em;
  font-weight: bold;
  margin-top: 20px;
  margin-bottom: 10px;
`

export const IntensitySelector: React.FC<IntensitySelectorProps> = ({
  value,
  min,
  max,
  showMaximumIntensity,
  maximumIntensity,
  onValueChange,
}) => {
  const handleChange = (newValue: number) => {
    onValueChange(newValue)
  }

  const handleSliderChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const newValue = Number(event.target.value)
    handleChange(newValue)
  }

  const sliderHeight = 350
  const maximumIntensityPositionInPixels = sliderHeight * (1 - (maximumIntensity - min) / (max - min))
  const maximumIntensityPosition = `${maximumIntensityPositionInPixels}px`

  return (
    <div>
      <SmallerTitle>Intensity</SmallerTitle>
      <IntensitySelectorContainer>
        {showMaximumIntensity && (
          <>
            <MaximumIntensityLine top={maximumIntensityPosition} />
            <MaximumIntensityLabel style={{ top: maximumIntensityPosition }}>
              <b>Max:</b> {maximumIntensity}
            </MaximumIntensityLabel>
          </>
        )}
        <input
          className='intensitySlider'
          type='range'
          min={min}
          max={max}
          value={value}
          onChange={handleSliderChange}
          style={{
            WebkitAppearance: 'slider-vertical' /* WebKit */,
            width: '8px',
            height: '350px',
            zIndex: 2,
          }}
        />
        <IntensityLabel>Intensity (V/m):</IntensityLabel>
        <ValidatedInput type='number' value={value} min={min} max={max} onChange={handleChange} />
      </IntensitySelectorContainer>
    </div>
  )
}
