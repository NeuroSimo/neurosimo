import React from 'react'
import styled from 'styled-components'

import { palette } from 'styles/General'

const Wrapper = styled.div`
  position: absolute;
  inset: 0;
  pointer-events: none;

  .arrow-vertical {
    width: 2px;
    background: ${palette.borderStrong};
    position: absolute;
    transform: translateX(-50%);
  }

  .arrow-vertical:after {
    content: '';
    position: absolute;
    left: 50%;
    bottom: -5px;
    transform: translateX(-50%);
    border-left: 5px solid transparent;
    border-right: 5px solid transparent;
    border-top: 6px solid ${palette.borderStrong};
  }
`

/* Spans run from the EEG source and node bottoms to the next node top (see PipelineDiagram). */
export const PipelineConnections: React.FC = () => (
  <Wrapper>
    <div className='arrow-vertical' style={{ left: '10%', top: '64px', height: '54px' }} />
    <div className='arrow-vertical' style={{ left: '10%', top: '170px', height: '55px' }} />
    <div className='arrow-vertical' style={{ left: '10%', top: '278px', height: '55px' }} />
  </Wrapper>
)

