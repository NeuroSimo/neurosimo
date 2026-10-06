import React, { useContext } from 'react'
import styled from 'styled-components'

import { EegSimulatorPanel } from 'components/EegSimulatorPanel'
import { ImportRecordingPanel } from 'components/ImportRecordingPanel'
import { RecordingsPanel } from 'components/RecordingsPanel'
import { EegDevicePanel } from 'components/EegDevicePanel'
import { EegStreamContext } from 'providers/EegStreamProvider'
import { useSessionConfig } from 'providers/SessionConfigProvider'
import { ConfigPanel, CONFIG_PANEL_WIDTH, ConfigTitle, palette } from 'styles/General'

// Context for sharing tab switching functionality
export const DataSourceContext = React.createContext<{
  setActiveTab: (tab: 'simulator' | 'recording' | 'eeg_device') => void
  activeTab: 'simulator' | 'recording' | 'eeg_device'
} | null>(null)

const DataSourcePanel = styled(ConfigPanel)`
  width: ${CONFIG_PANEL_WIDTH}px;
  height: auto;
  position: static;
  display: flex;
  flex-direction: column;
  gap: 0;
`

const TabContainer = styled.div`
  display: flex;
  gap: 2px;
  margin-bottom: 4px;
  border-bottom: 1px solid ${palette.border};
`

const Tab = styled.button<{ active: boolean; disabled?: boolean }>`
  padding: 4px 10px 5px 10px;
  margin-bottom: -1px;
  background: none;
  border: none;
  border-bottom: 2px solid ${props => props.active ? palette.blue : 'transparent'};
  color: ${props => props.disabled ? palette.textDim : props.active ? palette.text : palette.textMuted};
  font-weight: ${props => props.active ? 600 : 'normal'};
  font-family: inherit;
  cursor: ${props => props.disabled ? 'not-allowed' : 'pointer'};
  font-size: 12px;

  &:hover {
    color: ${props => props.disabled ? palette.textDim : palette.text};
  }
`

const StatusMessage = styled.div`
  font-size: 11px;
  font-weight: 600;
  color: ${palette.textMuted};
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