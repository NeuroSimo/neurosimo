import React, { useState } from 'react'
import styled from 'styled-components'

import { ControlsContainer, ControlBox, TopLabel, StyledRow } from 'styles/ExperimentStyles'

const StyledNarrowInput = styled.input`
  width: 35px;
  margin-right: 8px;
  padding: 4px;
`

const StyledButton = styled.button`
  padding: 8px 15px;
  border-radius: 4px;
  border: none;
  background-color: #007bff;
  color: white;
  cursor: pointer;
  transition: background-color 0.3s;

  &:hover {
    background-color: #0056b3;
  }
`

const CircleButton = styled(StyledButton)`
  margin-right: 8px;
`

const StyledLabel = styled.label`
  margin-right: 8px;
`

interface LocationControlsProps {
  onShapeSelected: (shape: 'circle' | 'square', size: number) => void
  onReset: () => void
  onDecimate: (rate: number) => void
}

export const LocationControls: React.FC<LocationControlsProps> = ({ onShapeSelected, onReset, onDecimate }) => {
  const [shapeSize, setShapeSize] = useState<number>(8)
  const [decimationRate, setDecimationRate] = useState<number>(2)

  return (
    <ControlsContainer>
      <ControlBox>
        <TopLabel>Shape</TopLabel>
        <StyledRow>
          <StyledLabel>Size:</StyledLabel>
          <StyledNarrowInput
            type='number'
            value={shapeSize}
            onChange={(e) => setShapeSize(Math.max(1, parseInt(e.target.value)))}
          />
        </StyledRow>
        <StyledRow>
          <CircleButton onClick={() => onShapeSelected('circle', shapeSize)}>Circle</CircleButton>
          <StyledButton onClick={() => onShapeSelected('square', shapeSize)}>Square</StyledButton>
        </StyledRow>
      </ControlBox>

      <ControlBox>
        <TopLabel>Decimation</TopLabel>
        <StyledRow>
          <StyledLabel>Rate:</StyledLabel>
          <StyledNarrowInput
            type='number'
            value={decimationRate}
            onChange={(e) => setDecimationRate(Math.max(1, parseInt(e.target.value)))}
          />
        </StyledRow>
        <StyledRow>
          <StyledButton onClick={() => onDecimate(decimationRate)}>Decimate</StyledButton>
        </StyledRow>
      </ControlBox>
      <ControlBox>
        <StyledButton onClick={onReset}>Reset</StyledButton>
      </ControlBox>
    </ControlsContainer>
  )
}
