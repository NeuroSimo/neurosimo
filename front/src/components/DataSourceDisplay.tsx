import React, { useContext } from 'react'
import styled from 'styled-components'

import { EegSimulatorPanel } from 'components/EegSimulatorPanel'
import { RecordingsPanel } from 'components/RecordingsPanel'
import { EegDevicePanel } from 'components/EegDevicePanel'
import { EegStreamContext } from 'providers/EegStreamProvider'
import { useSessionConfig } from 'providers/SessionConfigProvider'
import { StyledPanel, CONFIG_PANEL_WIDTH, SmallerTitle } from 'styles/General'

const DataSourcePanel = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH}px;
  height: 500px;
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
  const [activeTab, setActiveTab] = React.useState<'simulator' | 'recording' | 'eeg_device'>('simulator')
  const [previousTab, setPreviousTab] = React.useState<'simulator' | 'recording'>('simulator')
  const { eegDeviceInfo } = useContext(EegStreamContext)
  const { setDataSource } = useSessionConfig()

  const isEegStreaming = eegDeviceInfo?.is_streaming || false

  React.useEffect(() => {
    if (isEegStreaming) {
      // Remember the current tab before switching to EEG Device
      setPreviousTab(activeTab as 'simulator' | 'recording')
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
        <Tab active={activeTab === 'recording'} disabled={isEegStreaming} onClick={() => !isEegStreaming && setActiveTab('recording')}>
          Recordings
        </Tab>
        <Tab active={activeTab === 'eeg_device'} disabled={!isEegStreaming} onClick={() => isEegStreaming && setActiveTab('eeg_device')}>
          EEG Device
        </Tab>
      </TabContainer>

      {activeTab === 'simulator' && <EegSimulatorPanel isGrayedOut={false} />}
      {activeTab === 'recording' && <RecordingsPanel isGrayedOut={false} />}
      {activeTab === 'eeg_device' && <EegDevicePanel />}

      {isEegStreaming && activeTab === 'eeg_device' && (
        <StatusMessage>
          Live EEG stream detected.
        </StatusMessage>
      )}
    </DataSourcePanel>
  )
}