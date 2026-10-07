import React, { useContext } from 'react'
import styled from 'styled-components'

import { PreprocessorNode } from 'components/pipeline/PreprocessorNode'
import { DeciderNode } from 'components/pipeline/DeciderNode'
import { PresenterNode } from 'components/pipeline/PresenterNode'
import {
  PipelineConnection,
  PipelineElbow,
  PipelineHorizontalConnection,
} from 'components/pipeline/PipelineConnections'
import {
  PIPELINE_NODE_OUTER_HEIGHT,
  PIPELINE_NODE_OUTER_WIDTH,
  PIPELINE_NODE_SELECT_LEFT,
  PIPELINE_NODE_TITLE_INSET,
  PIPELINE_NODE_TOGGLE_RIGHT,
  PIPELINE_SCALE,
  scaled,
} from 'components/pipeline/PipelineNode'
import { ModuleListContext } from 'providers/ModuleListProvider'
import { palette } from 'styles/General'

/* The stages form one left-aligned column joined by vertical connectors through the nodes'
   horizontal centre. The EEG and TMS endpoints sit in the Decider's row, on either side of the
   node. */
const CONNECTOR_AXIS_X = PIPELINE_NODE_OUTER_WIDTH / 2
const ENDPOINT_SIZE = PIPELINE_NODE_TITLE_INSET * 2
const CONNECTOR_LENGTH = scaled(44)

const PipelinePanel = styled.div`
  display: flex;
  flex-direction: column;
  align-items: flex-start;
  position: relative;
`

const EndpointCircle = styled.div<{ $color: string; $tint: string }>`
  display: flex;
  justify-content: center;
  align-items: center;
  flex-shrink: 0;
  width: ${ENDPOINT_SIZE}px;
  height: ${ENDPOINT_SIZE}px;
  box-sizing: border-box;
  background-color: ${props => props.$tint};
  border: 1.5px solid ${props => props.$color};
  color: ${props => props.$color};
  border-radius: 50%;
  font-size: ${11 * PIPELINE_SCALE}px;
  font-weight: 700;
  letter-spacing: 0.04em;
  z-index: 10;
`

const EegCircle = styled(EndpointCircle)`
  cursor: move;
`

/* The EEG source and the Decider's stimulation output sit in the Decider's row, just left and right
   of the node. They are positioned outside the column's flow so the column's width and centring
   are unchanged. */
const DeciderRow = styled.div`
  position: relative;
`

const EegBranch = styled.div`
  position: absolute;
  top: 0;
  bottom: 0;
  right: 100%;
  width: ${ENDPOINT_SIZE + CONNECTOR_LENGTH}px;
  display: flex;
  align-items: center;
`

const TmsBranch = styled.div`
  position: absolute;
  top: 0;
  bottom: 0;
  left: 100%;
  display: flex;
  align-items: center;
`

interface FloatingTitleProps {
  xOffset: number;
  yOffset: number;
  $alignRight?: boolean;
}

const FloatingTitle = styled.div<FloatingTitleProps>`
  position: absolute;
  font-size: ${11 * PIPELINE_SCALE}px;
  font-weight: 500;
  color: ${palette.textMuted};
  pointer-events: none;
  z-index: 5;
  left: ${props => props.xOffset}px;
  top: ${props => props.yOffset}px;
  ${props => props.$alignRight && 'transform: translateX(-100%);'}
`

interface PipelineDiagramProps {
  enabledTitleX?: number;
  enabledTitleY?: number;
  moduleTitleX?: number;
  moduleTitleY?: number;
}

/* Column headings sit just above the Preprocessor node: "Enabled" ends at the toggle's right edge
   and "Module" starts at the module select's left edge. */
const HEADING_OFFSET_Y = -scaled(21)

/* With the Preprocessor enabled, EEG is routed from the top of the EEG source up and into the
   Preprocessor node's left edge (at its centre). Coordinates are relative to the Preprocessor's
   top-left corner. */
const PREPROCESSOR_MID_Y = PIPELINE_NODE_OUTER_HEIGHT / 2
const DECIDER_MID_Y = PIPELINE_NODE_OUTER_HEIGHT * 1.5 + CONNECTOR_LENGTH
const EEG_ROUTE_FROM_X = -(CONNECTOR_LENGTH + ENDPOINT_SIZE / 2)
const EEG_ROUTE_FROM_Y = DECIDER_MID_Y - ENDPOINT_SIZE / 2

export const PipelineDiagram: React.FC<PipelineDiagramProps> = ({
  enabledTitleX = PIPELINE_NODE_TOGGLE_RIGHT,
  enabledTitleY = HEADING_OFFSET_Y,
  moduleTitleX = PIPELINE_NODE_SELECT_LEFT,
  moduleTitleY = HEADING_OFFSET_Y,
}) => {
  const { preprocessorEnabled, deciderEnabled, presenterEnabled } = useContext(ModuleListContext)

  /* A connector is active when data flows along it: a disabled Decider processes no samples, and
     a disabled Presenter receives nothing. With the Preprocessor disabled, the Decider reads the
     EEG stream directly. */
  return (
    <PipelinePanel>
      <FloatingTitle xOffset={enabledTitleX} yOffset={enabledTitleY} $alignRight>
        Enabled
      </FloatingTitle>
      <FloatingTitle xOffset={moduleTitleX} yOffset={moduleTitleY}>
        Module
      </FloatingTitle>
      <PipelineElbow
        fromX={EEG_ROUTE_FROM_X}
        fromY={EEG_ROUTE_FROM_Y}
        toY={PREPROCESSOR_MID_Y}
        active={preprocessorEnabled}
      />
      <PreprocessorNode />
      <PipelineConnection
        axisX={CONNECTOR_AXIS_X}
        length={CONNECTOR_LENGTH}
        active={preprocessorEnabled && deciderEnabled}
      />
      <DeciderRow>
        <EegBranch>
          <EegCircle $color={palette.orange} $tint='rgba(229, 149, 74, 0.14)'>EEG</EegCircle>
          {!preprocessorEnabled && (
            <PipelineHorizontalConnection length={CONNECTOR_LENGTH} active={deciderEnabled} />
          )}
        </EegBranch>
        <DeciderNode />
        <TmsBranch>
          <PipelineHorizontalConnection length={CONNECTOR_LENGTH} active={deciderEnabled} />
          <EndpointCircle $color={palette.blue} $tint='rgba(74, 138, 212, 0.14)'>TMS</EndpointCircle>
        </TmsBranch>
      </DeciderRow>
      <PipelineConnection
        axisX={CONNECTOR_AXIS_X}
        length={CONNECTOR_LENGTH}
        active={deciderEnabled && presenterEnabled}
      />
      <PresenterNode />
    </PipelinePanel>
  )
}

