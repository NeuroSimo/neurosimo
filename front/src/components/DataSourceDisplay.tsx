import React, { useContext } from 'react'
import styled from 'styled-components'

import { EegSimulatorPanel } from 'components/EegSimulatorPanel'
import { PlaybackPanel } from 'components/PlaybackPanel'
import { EegDevicePanel } from 'components/EegDevicePanel'
import { EegStreamContext } from 'providers/EegStreamProvider'
import { useParameters } from 'providers/ParameterProvider'
import { StyledPanel, CONFIG_PANEL_WIDTH, SmallerTitle } from 'styles/General'

const DataSourcePanel = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH}px;
  height: 460px;
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

const Tab = styled.button<{ active: boolean; disabled?: boolean }>`
  padding: 0.2rem 0.4rem;
  background: none;
  border: none;
  border-bottom: 2px solid ${props => props.active ? '#007bff' : 'transparent'};
  color: ${props => props.disabled ? '#ccc' : props.active ? '#007bff' : '#666'};
  font-weight: ${props => props.active ? 'bold' : 'normal'};
  cursor: ${props => props.disabled ? 'not-allowed' : 'pointer'};
  font-size: 12px;

  &:hover {
    color: ${props => props.disabled ? '#ccc' : '#007bff'};
  }
`

const StatusMessage = styled.div`
  font-size: 10px;
  font-weight: bold;
  color: #666;
  text-align: center;
  margin-top: 4px;
  padding: 2px;
`

export const DataSourceDisplay: React.FC = () => {
  const [activeTab, setActiveTab] = React.useState<'simulator' | 'playback' | 'eeg_device'>('simulator')
  const [previousTab, setPreviousTab] = React.useState<'simulator' | 'playback'>('simulator')
  const { eegInfo } = useContext(EegStreamContext)
  const { setDataSource } = useParameters()

  const isEegStreaming = eegInfo?.is_streaming || false

  React.useEffect(() => {
    if (isEegStreaming) {
      // Remember the current tab before switching to EEG Device
      setPreviousTab(activeTab as 'simulator' | 'playback')
      setActiveTab('eeg_device')
    } else {
      // Restore the previous tab when streaming stops
      setActiveTab(previousTab)
    }
  }, [isEegStreaming])

  React.useEffect(() => {
    setDataSource(activeTab, () => {
      console.log('Data source set to ' + activeTab)
    })
  }, [activeTab])

  return (
    <DataSourcePanel>
      <SmallerTitle>Data Source</SmallerTitle>
      <TabContainer>
        <Tab active={activeTab === 'simulator'} disabled={isEegStreaming} onClick={() => !isEegStreaming && setActiveTab('simulator')}>
          Simulator
        </Tab>
        <Tab active={activeTab === 'playback'} disabled={isEegStreaming} onClick={() => !isEegStreaming && setActiveTab('playback')}>
          Playback
        </Tab>
        <Tab active={activeTab === 'eeg_device'} disabled={!isEegStreaming} onClick={() => isEegStreaming && setActiveTab('eeg_device')}>
          EEG Device
        </Tab>
      </TabContainer>

      {activeTab === 'simulator' && <EegSimulatorPanel isGrayedOut={false} />}
      {activeTab === 'playback' && <PlaybackPanel isGrayedOut={false} />}
      {activeTab === 'eeg_device' && <EegDevicePanel />}

      {isEegStreaming && activeTab === 'eeg_device' && (
        <StatusMessage>
          Live EEG stream detected.
        </StatusMessage>
      )}
    </DataSourcePanel>
  )
}