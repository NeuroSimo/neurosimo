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
  const [activeTab, setActiveTab] = React.useState<'simulator' | 'recording' | 'eeg_device'>('simulator')
  const [previousTab, setPreviousTab] = React.useState<'simulator' | 'recording'>('simulator')
  const [hasInitialized, setHasInitialized] = React.useState(false)
  const { eegDeviceInfo } = useContext(EegStreamContext)
  const { dataSource, setDataSource } = useSessionConfig()

  const isEegStreaming = eegDeviceInfo?.is_streaming || false

  // Sync activeTab from session config's dataSource (e.g., when switching projects)
  // This runs when dataSource changes from backend (project load/switch)
  React.useEffect(() => {
    if (!dataSource) return

    const configDataSource = dataSource as 'simulator' | 'recording' | 'eeg_device'
    
    // Determine the effective tab based on device availability
    let effectiveTab: 'simulator' | 'recording' | 'eeg_device'
    if (configDataSource === 'eeg_device' && !isEegStreaming) {
      // EEG device requested but not available - fallback to simulator
      effectiveTab = 'simulator'
    } else {
      effectiveTab = configDataSource
    }

    // Sync activeTab from session config
    if (effectiveTab !== activeTab) {
      setActiveTab(effectiveTab)
      if (effectiveTab !== 'eeg_device') {
        setPreviousTab(effectiveTab)
      }
    }

    // If session config doesn't match the effective tab, update it
    // This handles the case where eeg_device is in config but device isn't available
    if (configDataSource !== effectiveTab) {
      setDataSource(effectiveTab, () => {
        console.log('Data source corrected to ' + effectiveTab)
      })
    }
    
    setHasInitialized(true)
  }, [dataSource, isEegStreaming])

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

  // Sync session config when user manually changes tab (after initialization)
  React.useEffect(() => {
    if (!hasInitialized) return
    
    // Only update if the current tab differs from what's in the session config
    if (activeTab !== dataSource) {
      setDataSource(activeTab, () => {
        console.log('Data source set to ' + activeTab)
      })
    }
  }, [activeTab, hasInitialized])

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