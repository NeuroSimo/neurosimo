import React from 'react'
import styled from 'styled-components'

import { palette } from 'styles/General'
import { scaled } from 'components/pipeline/PipelineNode'

const ARROWHEAD_HEIGHT = scaled(6)
const ARROWHEAD_HALF_WIDTH = scaled(5)
/* Active connectors carry data and are drawn bright with an arrowhead. Connectors that carry no
   data in the current configuration keep only a thin, faint structural line without an arrowhead. */
const STROKE_COLOR = palette.textMuted
const ACTIVE_STROKE_WIDTH = 2
const INACTIVE_STROKE_WIDTH = 1
const INACTIVE_OPACITY = 0.25

const strokeWidth = (active: boolean) => active ? ACTIVE_STROKE_WIDTH : INACTIVE_STROKE_WIDTH

/* A downward arrow placed in the pipeline column between two stages. It fills its whole height,
   so the line starts at the stage above and the arrowhead tip touches the stage below. */
const Connector = styled.div<{ $axisX: number; $length: number; $active: boolean }>`
  position: relative;
  flex-shrink: 0;
  width: ${ARROWHEAD_HALF_WIDTH * 2}px;
  height: ${props => props.$length}px;
  margin-left: ${props => props.$axisX - ARROWHEAD_HALF_WIDTH}px;
  pointer-events: none;
  opacity: ${props => props.$active ? 1 : INACTIVE_OPACITY};
  transition: opacity 0.2s;

  &:before {
    content: '';
    position: absolute;
    top: 0;
    bottom: ${props => props.$active ? ARROWHEAD_HEIGHT : 0}px;
    left: 50%;
    width: ${props => strokeWidth(props.$active)}px;
    transform: translateX(-50%);
    background: ${STROKE_COLOR};
  }

  &:after {
    content: '';
    display: ${props => props.$active ? 'block' : 'none'};
    position: absolute;
    bottom: 0;
    left: 0;
    border-left: ${ARROWHEAD_HALF_WIDTH}px solid transparent;
    border-right: ${ARROWHEAD_HALF_WIDTH}px solid transparent;
    border-top: ${ARROWHEAD_HEIGHT}px solid ${STROKE_COLOR};
  }
`

interface PipelineConnectionProps {
  axisX: number
  length: number
  active: boolean
}

export const PipelineConnection: React.FC<PipelineConnectionProps> = ({ axisX, length, active }) => (
  <Connector $axisX={axisX} $length={length} $active={active} />
)

/* A rightward arrow in a row, from the stage on its left to the endpoint on its right. */
const HorizontalConnector = styled.div<{ $length: number; $active: boolean }>`
  position: relative;
  flex-shrink: 0;
  width: ${props => props.$length}px;
  height: ${ARROWHEAD_HALF_WIDTH * 2}px;
  pointer-events: none;
  opacity: ${props => props.$active ? 1 : INACTIVE_OPACITY};
  transition: opacity 0.2s;

  &:before {
    content: '';
    position: absolute;
    left: 0;
    right: ${props => props.$active ? ARROWHEAD_HEIGHT : 0}px;
    top: 50%;
    height: ${props => strokeWidth(props.$active)}px;
    transform: translateY(-50%);
    background: ${STROKE_COLOR};
  }

  &:after {
    content: '';
    display: ${props => props.$active ? 'block' : 'none'};
    position: absolute;
    right: 0;
    top: 0;
    border-top: ${ARROWHEAD_HALF_WIDTH}px solid transparent;
    border-bottom: ${ARROWHEAD_HALF_WIDTH}px solid transparent;
    border-left: ${ARROWHEAD_HEIGHT}px solid ${STROKE_COLOR};
  }
`

interface PipelineHorizontalConnectionProps {
  length: number
  active: boolean
}

export const PipelineHorizontalConnection: React.FC<PipelineHorizontalConnectionProps> = ({ length, active }) => (
  <HorizontalConnector $length={length} $active={active} />
)

/* Zero-size anchor at the pipeline's top-left corner; the route is drawn relative to it. */
const ElbowSvg = styled.svg<{ $active: boolean }>`
  position: absolute;
  top: 0;
  left: 0;
  width: 1px;
  height: 1px;
  overflow: visible;
  pointer-events: none;
  opacity: ${props => props.$active ? 1 : INACTIVE_OPACITY};
  transition: opacity 0.2s;
`

interface PipelineElbowProps {
  /* Start point, relative to the pipeline's top-left corner, and the vertical position where the
     route enters the target's left edge at x = 0. */
  fromX: number
  fromY: number
  toY: number
  active: boolean
}

/* A connector that runs vertically from a source outside the column, then turns right into the
   target's left edge. */
export const PipelineElbow: React.FC<PipelineElbowProps> = ({ fromX, fromY, toY, active }) => (
  <ElbowSvg $active={active} aria-hidden>
    <path
      d={`M ${fromX} ${fromY} V ${toY} H ${active ? -ARROWHEAD_HEIGHT : 0}`}
      fill='none'
      stroke={STROKE_COLOR}
      strokeWidth={strokeWidth(active)}
      strokeLinejoin='round'
    />
    {active && (
      <polygon
        points={
          `${-ARROWHEAD_HEIGHT},${toY - ARROWHEAD_HALF_WIDTH} 0,${toY} ${-ARROWHEAD_HEIGHT},${toY + ARROWHEAD_HALF_WIDTH}`
        }
        fill={STROKE_COLOR}
      />
    )}
  </ElbowSvg>
)

