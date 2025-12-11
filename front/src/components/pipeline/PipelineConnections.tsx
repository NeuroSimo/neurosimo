import React from 'react'
import styled from 'styled-components'

const Wrapper = styled.div`
  position: absolute;
  inset: 0;
  pointer-events: none;

  .arrow-vertical {
    width: 5px;
    height: 45px;
    background: #888;
    position: absolute;
    transform: translateX(-50%);
  }

  .arrow-vertical:after {
    content: '';
    position: absolute;
    left: 50%;
    bottom: -5px;
    transform: translateX(-50%);
    border-left: 6px solid transparent;
    border-right: 6px solid transparent;
    border-top: 6px solid #888;
  }
`

export const PipelineConnections: React.FC = () => (
  <Wrapper>
    <div className='arrow-vertical' style={{ left: '10%', top: '59px' }} />
    <div className='arrow-vertical' style={{ left: '10%', top: '166px' }} />
    <div className='arrow-vertical' style={{ left: '10%', top: '274px' }} />
  </Wrapper>
)

