import React, { useContext } from 'react'
import styled from 'styled-components'

import { EegSimulatorPanel } from 'components/EegSimulatorPanel'
import { ImportRecordingPanel } from 'components/ImportRecordingPanel'
import { RecordingsPanel } from 'components/RecordingsPanel'
import { EegDevicePanel } from 'components/EegDevicePanel'
import { EegStreamContext } from 'providers/EegStreamProvider'
import { useSessionConfig } from 'providers/SessionConfigProvider'
import { ConfigPanel, CONFIG_PANEL_WIDTH, ConfigTitle } from 'styles/General'

// Context for sharing tab switching functionality
export const DataSourceContext = React.createContext<{
  setActiveTab: (tab: 'simulator' | 'recording' | 'eeg_device') => void
  activeTab: 'simulator' | 'recording' | 'eeg_device'
} | null>(null)

const DataSourcePanel = styled(ConfigPanel)`
  width: ${CONFIG_PANEL_WIDTH}px;
  height: auto;
  min-height: 710px;
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
  const { eegDeviceInfo } = useContext(EegStreamContext)
  const { setDataSource } = useSessionConfig()

  const isEegStreaming = eegDeviceInfo?.is_streaming || false

  // The UI is the single source of truth for the data source. It is local state,
  // initialized to 'simulator', and is deliberately never restored from the backend
  // session config: switching projects keeps whatever tab is currently shown.
  const [activeTab, setActiveTab] = React.useState<'simulator' | 'recording' | 'eeg_device'>('simulator')
  // Remembers the simulator/recording choice so it can be restored when EEG
  // streaming stops (an active stream forces the 'eeg_device' tab).
  const [previousTab, setPreviousTab] = React.useState<'simulator' | 'recording'>('simulator')

  // EEG streaming forces the 'eeg_device' tab; restore the previous tab when it stops.
  React.useEffect(() => {
    if (isEegStreaming) {
      if (activeTab !== 'eeg_device') {
        setPreviousTab(activeTab)
      }
      setActiveTab('eeg_device')
    } else {
      setActiveTab(previousTab)
    }
  }, [isEegStreaming])

  // Push the currently shown data source to the backend so the session manager
  // uses exactly what the UI displays.
  React.useEffect(() => {
    setDataSource(activeTab)
  }, [activeTab])

  return (
    <DataSourceContext.Provider value={{ setActiveTab, activeTab }}>
      <DataSourcePanel>
        <ConfigTitle>Data Source</ConfigTitle>
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
        {activeTab === 'simulator' && <ImportRecordingPanel />}
        {activeTab === 'recording' && <RecordingsPanel isGrayedOut={false} />}
        {activeTab === 'eeg_device' && <EegDevicePanel />}

        {isEegStreaming && activeTab === 'eeg_device' && (
          <StatusMessage>
            Live EEG stream detected.
          </StatusMessage>
        )}
      </DataSourcePanel>
    </DataSourceContext.Provider>
  )
}