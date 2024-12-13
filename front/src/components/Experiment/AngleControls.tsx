import React, { useState } from 'react'
import styled from 'styled-components'

import { ControlsContainer, ControlBox, TopLabel, StyledRow } from 'styles/ExperimentStyles'

const StyledNarrowInput = styled.input`
  width: 45px;
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
  margin-bottom: 5px;
`

export const AngleControls: React.FC<{
  onFillAngles: (startAngle: number, increment: number) => void
  onReset: () => void
}> = ({ onFillAngles, onReset }) => {
  const [startAngle, setStartAngle] = useState<number>(0)
  const [increment, setIncrement] = useState<number>(30)

  return (
    <ControlsContainer>
      <ControlBox>
        <TopLabel>Angles</TopLabel>
        <StyledRow>
          <StyledLabel>Start Angle:</StyledLabel>
          <StyledNarrowInput
            type='number'
            value={startAngle}
            onChange={(e) => setStartAngle(Math.max(0, Math.min(350, parseInt(e.target.value))))}
          />
        </StyledRow>
        <StyledRow>
          <StyledLabel>Increment:</StyledLabel>
          <StyledNarrowInput
            type='number'
            value={increment}
            onChange={(e) => setIncrement(Math.max(0, Math.min(350, parseInt(e.target.value))))}
          />
        </StyledRow>
        <StyledRow>
          <StyledButton onClick={() => onFillAngles(startAngle, increment)}>Fill</StyledButton>
        </StyledRow>
      </ControlBox>

      <ControlBox>
        <StyledButton onClick={onReset}>Reset</StyledButton>
      </ControlBox>
    </ControlsContainer>
  )
}
