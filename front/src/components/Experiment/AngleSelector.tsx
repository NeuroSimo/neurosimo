import React, { useState } from 'react'
import styled from 'styled-components'

import { AngleControls } from './AngleControls'
import { LargerTitle } from 'styles/ExperimentStyles'

type SetSelectedAngles = React.Dispatch<React.SetStateAction<number[]>>

interface AngleSelectorProps {
  selectedAngles: number[]
  setSelectedAngles: SetSelectedAngles
  highlightedAngles: number[]
  multiSelectMode?: boolean
}

interface SectorProps {
  isSelected: boolean
  isHighlighted: boolean
}

const SelectorInternalContainer = styled.div`
  display: flex;
  gap: 20px;
`

const SelectorContainer = styled.div`
  position: relative;
  width: max-content;
`

const SVGContainer = styled.svg``

const Sector = styled.path<SectorProps>`
  fill: transparent;
  stroke: #ccc;
  stroke-width: 2;
  cursor: pointer;
  transition: fill 0.2s;

  fill: ${(props) => {
    if (props.isHighlighted) return '#000'
    if (props.isSelected) return '#007bff'
    return 'transparent'
  }};

  // Hover effect only when the sector is not selected
  &:hover {
    fill: ${(props) => (props.fill !== 'blue' ? 'rgba(0, 123, 255, 0.4)' : '#007bff')};
  }
`

const AngleLabel = styled.text`
  font-size: 12px;
  text-anchor: middle;
  transition: color 0.2s;
`

/* Large angle text shown next to the grid only when not in multiSelectMode. */
const LargeAngleText = styled.div`
  margin-top: 160px;
  margin-left: 50px;
  font-size: 36px;
  font-weight: bold;
  text-anchor: middle;
  fill: black;
  opacity: 0;
  transition: opacity 0.5s;

  &[data-visible='true'] {
    opacity: 1;
  }
`

export const AngleSelector: React.FC<AngleSelectorProps> = ({
  selectedAngles,
  setSelectedAngles,
  highlightedAngles,
  multiSelectMode,
}) => {
  const [isDragging, setIsDragging] = useState<boolean>(false)
  const [dragMode, setDragMode] = useState<'selecting' | 'deselecting' | null>(null)
  const [hoveredAngle, setHoveredAngle] = useState<number | null>(null)

  const handleFillAngles = (startAngle: number, increment: number) => {
    const angles: number[] = []
    let currentAngle = startAngle
    while (currentAngle < 360) {
      angles.push(currentAngle)
      currentAngle += increment
    }
    setSelectedAngles(angles)
  }

  const handleResetAngles = () => {
    setSelectedAngles([])
  }

  const handleSectorMouseDown = (angle: number) => {
    if (multiSelectMode) {
      if (selectedAngles.includes(angle)) {
        setSelectedAngles((prev) => prev.filter((a) => a !== angle))
        setDragMode('deselecting')
      } else {
        setSelectedAngles((prev) => [...prev, angle])
        setDragMode('selecting')
      }
    } else {
      if (selectedAngles.includes(angle)) {
        setSelectedAngles([])
      } else {
        setSelectedAngles([angle])
      }
    }
    setIsDragging(true)
  }

  const handleSectorMouseEnter = (angle: number) => {
    setHoveredAngle(angle)
    if (isDragging) {
      if (multiSelectMode) {
        if (dragMode === 'selecting' && !selectedAngles.includes(angle)) {
          setSelectedAngles((prev) => [...prev, angle])
        } else if (dragMode === 'deselecting' && selectedAngles.includes(angle)) {
          setSelectedAngles((prev) => prev.filter((a) => a !== angle))
        }
      } else {
        // Use the same logic as mouse down but without setting isDragging
        if (!selectedAngles.includes(angle)) {
          setSelectedAngles([angle])
        }
      }
    }
  }

  const handleMouseUp = () => {
    setIsDragging(false)
    setDragMode(null)
  }

  const isAngleSelected = (angle: number) => selectedAngles.includes(angle)
  const isAngleHighlighted = (angle: number) => highlightedAngles.includes(angle) && selectedAngles.length > 1

  return (
    <div>
      <LargerTitle>Angle</LargerTitle>
      <SelectorInternalContainer>
        <SelectorContainer onMouseUp={handleMouseUp}>
          <SVGContainer width='400' height='400' viewBox='0 0 400 400' onMouseLeave={() => setHoveredAngle(null)}>
            <circle cx='200' cy='200' r='150' fill='lightgray' />
            {Array.from({ length: 36 }).map((_, index) => {
              const angle = index * 10
              const isSelected = isAngleSelected(angle)
              const isHighlighted = isAngleHighlighted(angle)
              const labelX = 200 + 170 * Math.cos((angle * Math.PI) / 180)
              const labelY = 200 + 170 * Math.sin((angle * Math.PI) / 180)

              return (
                  <g key={angle} transform={'rotate(-90 200 200)'}>
                    <AngleLabel
                      x={labelX}
                      y={labelY}
                      fill={isSelected ? 'black' : '#bbb'}
                      transform={`rotate(90 ${labelX} ${labelY})`} // Counter-rotate the label
                    >
                    {angle}°
                  </AngleLabel>
                  <Sector
                    d={`M 200 200 L ${200 + 150 * Math.cos(((angle - 5) * Math.PI) / 180)} ${
                      200 + 150 * Math.sin(((angle - 5) * Math.PI) / 180)
                    } A 150 150 0 0 1 ${200 + 150 * Math.cos(((angle + 5) * Math.PI) / 180)} ${
                      200 + 150 * Math.sin(((angle + 5) * Math.PI) / 180)
                    } Z`}
                    isSelected={isSelected}
                    isHighlighted={isHighlighted}
                    onMouseDown={() => handleSectorMouseDown(angle)}
                    onMouseEnter={() => handleSectorMouseEnter(angle)}
                  />
                </g>
              )
            })}
          </SVGContainer>
        </SelectorContainer>
        {!multiSelectMode && (
          <LargeAngleText data-visible={selectedAngles.length > 0}>
            {(selectedAngles.length > 0 || hoveredAngle !== null) &&
              `${selectedAngles.length > 0 ? selectedAngles[0] : hoveredAngle}°`}
          </LargeAngleText>
        )}
        {multiSelectMode && <AngleControls onFillAngles={handleFillAngles} onReset={handleResetAngles} />}
      </SelectorInternalContainer>
    </div>
  )
}
