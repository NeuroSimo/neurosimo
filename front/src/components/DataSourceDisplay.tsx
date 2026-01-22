import React, { useContext } from 'react'
import styled from 'styled-components'

import { EegSimulatorPanel } from 'components/EegSimulatorPanel'
import { PlaybackPanel } from 'components/PlaybackPanel'
import { EegStreamContext } from 'providers/EegStreamProvider'
import { StyledPanel, CONFIG_PANEL_WIDTH, SmallerTitle } from 'styles/General'

const DataSourcePanel = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH}px;
  height: 280px;
  position: static;
  display: flex;
  flex-direction: column;
  gap: 4px;
`

const TabContainer = styled.div`
  display: flex;
  padding: 0.2rem 0.2rem 0 0.2rem;
  border-bottom: 1px solid #ddd;
`

const Tab = styled.button<{ active: boolean }>`
  padding: 0.2rem 0.4rem;
  background: none;
  border: none;
  border-bottom: 2px solid ${props => props.active ? '#007bff' : 'transparent'};
  color: ${props => props.active ? '#007bff' : '#666'};
  font-weight: ${props => props.active ? 'bold' : 'normal'};
  cursor: pointer;
  font-size: 12px;

  &:hover {
    color: #007bff;
  }
`

export const DataSourceDisplay: React.FC = () => {
  const [activeTab, setActiveTab] = React.useState<'simulator' | 'playback'>('simulator')
  const { eegInfo } = useContext(EegStreamContext)

  const isEegStreaming = eegInfo?.is_streaming || false

  return (
    <DataSourcePanel isGrayedOut={isEegStreaming}>
      <SmallerTitle>Data Source</SmallerTitle>
      <TabContainer>
        <Tab active={activeTab === 'simulator'} onClick={() => setActiveTab('simulator')}>
          Simulator
        </Tab>
        <Tab active={activeTab === 'playback'} onClick={() => setActiveTab('playback')}>
          Playback
        </Tab>
      </TabContainer>

      {activeTab === 'simulator' && <EegSimulatorPanel isGrayedOut={false} />}
      {activeTab === 'playback' && <PlaybackPanel isGrayedOut={false} />}
    </DataSourcePanel>
  )
}