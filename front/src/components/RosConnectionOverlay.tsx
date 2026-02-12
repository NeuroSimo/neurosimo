import React from 'react'
import styled from 'styled-components'
import { useRosConnection } from 'providers/RosConnectionProvider'

const Overlay = styled.div<{ isVisible: boolean }>`
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: rgba(64, 64, 64, 0.8);
  display: ${props => props.isVisible ? 'flex' : 'none'};
  align-items: center;
  justify-content: center;
  z-index: 9999;
  color: white;
  font-size: 18px;
  font-weight: bold;
  text-align: center;
  padding: 20px;
`

const Message = styled.div`
  background-color: rgba(0, 0, 0, 0.7);
  padding: 20px 30px;
  border-radius: 8px;
  border: 2px solid #666;
`

export const RosConnectionOverlay: React.FC = () => {
  const { isConnected } = useRosConnection()

  return (
    <Overlay isVisible={!isConnected}>
      <Message>
        <div>🔌 Not connected to backend</div>
        <div style={{ fontSize: '14px', marginTop: '8px', opacity: 0.8 }}>
          Attempting to reconnect...
        </div>
      </Message>
    </Overlay>
  )
}