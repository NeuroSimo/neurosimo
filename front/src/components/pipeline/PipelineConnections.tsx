import React from 'react'
import styled from 'styled-components'

import { palette } from 'styles/General'

const ARROWHEAD_HEIGHT = 6
const ARROWHEAD_HALF_WIDTH = 5

/* A downward arrow placed in the pipeline column between two stages. It fills its whole height,
   so the line starts at the stage above and the arrowhead tip touches the stage below. */
const Connector = styled.div<{ $axisX: number; $length: number }>`
  position: relative;
  flex-shrink: 0;
  width: ${ARROWHEAD_HALF_WIDTH * 2}px;
  height: ${props => props.$length}px;
  margin-left: ${props => props.$axisX - ARROWHEAD_HALF_WIDTH}px;
  pointer-events: none;

  &:before {
    content: '';
    position: absolute;
    top: 0;
    bottom: ${ARROWHEAD_HEIGHT}px;
    left: 50%;
    width: 2px;
    transform: translateX(-50%);
    background: ${palette.borderStrong};
  }

  &:after {
    content: '';
    position: absolute;
    bottom: 0;
    left: 0;
    border-left: ${ARROWHEAD_HALF_WIDTH}px solid transparent;
    border-right: ${ARROWHEAD_HALF_WIDTH}px solid transparent;
    border-top: ${ARROWHEAD_HEIGHT}px solid ${palette.borderStrong};
  }
`

interface PipelineConnectionProps {
  axisX: number
  length: number
}

export const PipelineConnection: React.FC<PipelineConnectionProps> = ({ axisX, length }) => (
  <Connector $axisX={axisX} $length={length} />
)

