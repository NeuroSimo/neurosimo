import styled from 'styled-components'

/* Larger title, used for 'Position' and 'Angle' titles. */
export const LargerTitle = styled.h2`
  font-size: 24px;
  text-align: center;
  color: #333;
  margin-bottom: 30px;
  margin-right: 200px;
  font-weight: bold;
`

/* Smaller title, used for 'Intensity' title. */
export const SmallerTitle = styled.h2`
  font-size: 18px;
  text-align: center;
  margin-bottom: 30px;
  margin-right: 30px;
  font-weight: bold;
`

/* ControlsContainer, ControlBox, TopLabel, and StyledRow are used for grid and angle controls. */
export const ControlsContainer = styled.div`
  display: flex;
  flex-direction: column;
  gap: 10px;
`

export const ControlBox = styled.div`
  border: 4px solid #e0e0e0;
  padding: 10px;
  border-radius: 5px;
  margin-right: 8px;
  display: flex;
  flex-direction: column;
  align-items: center;
`

export const TopLabel = styled.div`
  font-weight: bold;
  margin-bottom: 8px;
  text-align: center;
  width: 100%;
`

export const StyledRow = styled.div`
  display: flex;
  align-items: center;
  justify-content: center;
  margin-bottom: 8px;
  width: 100%;
  flex-wrap: wrap;
`
