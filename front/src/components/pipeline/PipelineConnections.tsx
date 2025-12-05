import React from 'react'
import styled from 'styled-components'

const Wrapper = styled.div`
  position: absolute;
  inset: 0;
  pointer-events: none;

  .arrow {
    width: 18px;
    height: 5px;
    background: #888;
    position: absolute;
    transform: translateY(-50%);
  }

  .arrow:after {
    content: '';
    position: absolute;
    top: 50%;
    right: -5px;
    transform: translateY(-50%);
    border-top: 6px solid transparent;
    border-bottom: 6px solid transparent;
    border-left: 6px solid #888;
  }
`

export const PipelineConnections: React.FC = () => (
  <Wrapper>
    <div className='arrow' style={{ left: '47px', top: '68px' }} />
    <div className='arrow' style={{ left: '252px', top: '68px' }} />
    <div className='arrow' style={{ left: '457px', top: '68px' }} />
    <div className='arrow' style={{ left: '453px', top: '146px', width: '27px', transform: 'rotate(45deg)' }} />
  </Wrapper>
)

