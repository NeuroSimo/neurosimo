import React, { useState, useEffect } from 'react'
import styled from 'styled-components'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'
import { faTimes, faInfoCircle } from '@fortawesome/free-solid-svg-icons'
import { useGlobalConfig } from 'providers/GlobalConfigProvider'
import { ValidatedInput } from './ValidatedInput'

const ModalOverlay = styled.div`
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: rgba(0, 0, 0, 0.5);
  display: flex;
  justify-content: center;
  align-items: center;
  z-index: 1100;
`

const ModalContent = styled.div`
  background: white;
  border-radius: 8px;
  padding: 20px;
  max-width: 650px;
  width: 90%;
  height: 600px;
  display: flex;
  flex-direction: column;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
`

const ModalHeader = styled.div`
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 16px;
  border-bottom: 1px solid #e0e0e0;
  padding-bottom: 12px;
`

const ModalTitle = styled.h3`
  margin: 0;
  color: #333;
`

const CloseButton = styled.button`
  background: none;
  border: none;
  font-size: 18px;
  cursor: pointer;
  color: #666;
  &:hover {
    color: #333;
  }
`

const TabContainer = styled.div`
  display: flex;
  border-bottom: 2px solid #e0e0e0;
  margin-bottom: 20px;
  gap: 4px;
`

const Tab = styled.button<{ active: boolean }>`
  padding: 10px 20px;
  background: ${props => props.active ? '#007bff' : 'transparent'};
  color: ${props => props.active ? 'white' : '#555'};
  border: none;
  border-bottom: 2px solid ${props => props.active ? '#007bff' : 'transparent'};
  cursor: pointer;
  font-size: 14px;
  font-weight: 500;
  transition: all 0.2s;
  border-radius: 4px 4px 0 0;

  &:hover {
    background: ${props => props.active ? '#0056b3' : '#f0f0f0'};
  }
`

const TabContent = styled.div`
  flex: 1;
  overflow-y: auto;
  padding-right: 8px;
`

const Form = styled.form`
  display: flex;
  flex-direction: column;
  height: 100%;
  gap: 12px;
`

const Section = styled.div`
  display: flex;
  flex-direction: column;
  gap: 16px;
`

const InputGroup = styled.div`
  display: flex;
  flex-direction: column;
  gap: 6px;
`

const LabelRow = styled.div`
  display: flex;
  align-items: center;
  gap: 6px;
`

const Label = styled.label`
  font-weight: 500;
  color: #555;
  font-size: 13px;
`

const InfoIconWrapper = styled.span`
  position: relative;
  display: inline-flex;
  align-items: center;
  cursor: help;
`

const InfoIcon = styled(FontAwesomeIcon)`
  color: #666;
  font-size: 13px;
  
  &:hover {
    color: #007bff;
  }
`

const Tooltip = styled.div<{ show: boolean }>`
  position: absolute;
  bottom: 100%;
  left: 50%;
  transform: translateX(-50%);
  margin-bottom: 8px;
  padding: 8px 12px;
  background: #333;
  color: white;
  font-size: 12px;
  border-radius: 4px;
  white-space: normal;
  max-width: 250px;
  width: max-content;
  z-index: 1000;
  pointer-events: none;
  opacity: ${props => props.show ? 1 : 0};
  visibility: ${props => props.show ? 'visible' : 'hidden'};
  transition: opacity 0.2s, visibility 0.2s;
  
  &::after {
    content: '';
    position: absolute;
    top: 100%;
    left: 50%;
    transform: translateX(-50%);
    border: 6px solid transparent;
    border-top-color: #333;
  }
`

const Input = styled.input`
  padding: 8px 12px;
  border: 1px solid #ddd;
  border-radius: 4px;
  font-size: 14px;
  &:focus {
    outline: none;
    border-color: #007bff;
    box-shadow: 0 0 0 2px rgba(0, 123, 255, 0.25);
  }
`

const Select = styled.select`
  padding: 8px 12px;
  border: 1px solid #ddd;
  border-radius: 4px;
  font-size: 14px;
  background-color: white;
  &:focus {
    outline: none;
    border-color: #007bff;
    box-shadow: 0 0 0 2px rgba(0, 123, 255, 0.25);
  }
`

const Checkbox = styled.input`
  width: 18px;
  height: 18px;
  cursor: pointer;
`

const CheckboxWrapper = styled.div`
  display: flex;
  align-items: center;
  gap: 8px;
`

const ButtonGroup = styled.div`
  display: flex;
  justify-content: flex-end;
  gap: 12px;
  margin-top: 8px;
  padding-top: 12px;
  border-top: 1px solid #e0e0e0;
`

const Button = styled.button<{ variant?: 'primary' | 'secondary' }>`
  padding: 8px 16px;
  border: 1px solid ${props => props.variant === 'primary' ? '#007bff' : '#6c757d'};
  border-radius: 4px;
  background-color: ${props => props.variant === 'primary' ? '#007bff' : '#6c757d'};
  color: white;
  cursor: pointer;
  font-size: 14px;
  font-weight: 500;
  transition: background-color 0.2s;

  &:hover {
    background-color: ${props => props.variant === 'primary' ? '#0056b3' : '#545b62'};
  }

  &:disabled {
    opacity: 0.6;
    cursor: not-allowed;
  }
`

interface GlobalConfigModalProps {
  isOpen: boolean
  onClose: () => void
}

interface GlobalConfigValues {
  // EEG Configuration
  eegPort: number
  eegDevice: string
  turbolinkSamplingFrequency: number
  turbolinkEegChannelCount: number
  maximumDroppedSamples: number
  
  // LabJack Configuration
  simulateLabjack: boolean
  
  // Safety Configuration
  minimumIntertrialInterval: number

  // Timing Configuration
  maximumLoopbackLatency: number
  maximumTimingError: number
  triggerToPulseDelay: number
  
  // Disk Space Monitoring Configuration
  diskWarningThreshold: number
  diskErrorThreshold: number
  
  // System Configuration
  locale: string
}

const emptyConfig: GlobalConfigValues = {
  // EEG Configuration
  eegPort: 0,
  eegDevice: '',
  turbolinkSamplingFrequency: 0,
  turbolinkEegChannelCount: 0,
  maximumDroppedSamples: 0,
  
  // LabJack Configuration
  simulateLabjack: false,
  
  // Safety Configuration
  minimumIntertrialInterval: 0,

  // Timing Configuration
  maximumLoopbackLatency: 0,
  maximumTimingError: 0,
  triggerToPulseDelay: 0,
  
  // Disk Space Monitoring Configuration
  diskWarningThreshold: 0,
  diskErrorThreshold: 0,
  
  // System Configuration
  locale: '',
}

type TabType = 'eeg' | 'labjack' | 'timing' | 'safety' | 'system'

const COMMON_LOCALES = [
  { value: 'zh-CN', label: 'Chinese Simplified (简体中文)' },
  { value: 'da-DK', label: 'Danish (Dansk)' },
  { value: 'nl-NL', label: 'Dutch (Nederlands)' },
  { value: 'en-GB', label: 'English (UK)' },
  { value: 'en-US', label: 'English (US)' },
  { value: 'fi-FI', label: 'Finnish (Suomi)' },
  { value: 'fr-FR', label: 'French (Français)' },
  { value: 'de-DE', label: 'German (Deutsch)' },
  { value: 'it-IT', label: 'Italian (Italiano)' },
  { value: 'ja-JP', label: 'Japanese (日本語)' },
  { value: 'ko-KR', label: 'Korean (한국어)' },
  { value: 'nb-NO', label: 'Norwegian (Norsk)' },
  { value: 'pl-PL', label: 'Polish (Polski)' },
  { value: 'ru-RU', label: 'Russian (Русский)' },
  { value: 'es-ES', label: 'Spanish (Español)' },
  { value: 'sv-SE', label: 'Swedish (Svenska)' },
]

export const GlobalConfigModal: React.FC<GlobalConfigModalProps> = ({
  isOpen,
  onClose,
}) => {
  const globalConfig = useGlobalConfig()
  const [config, setConfig] = useState<GlobalConfigValues>(emptyConfig)
  const [initialConfig, setInitialConfig] = useState<GlobalConfigValues>(emptyConfig)
  const [activeTab, setActiveTab] = useState<TabType>('eeg')

  // Helper to parse disk threshold from string (e.g., "100GiB" -> 100)
  const parseDiskThreshold = (value: string): number => {
    if (!value) return 0
    const match = value.match(/^(\d+)/)
    return match ? parseInt(match[1]) : 0
  }

  // Check if config has been modified
  const hasChanges = (): boolean => {
    return JSON.stringify(config) !== JSON.stringify(initialConfig)
  }

  // Load config when modal opens
  useEffect(() => {
    if (isOpen && globalConfig.eegDevice) {
      const loadedConfig = {
        eegPort: globalConfig.eegPort,
        eegDevice: globalConfig.eegDevice,
        turbolinkSamplingFrequency: globalConfig.turbolinkSamplingFrequency,
        turbolinkEegChannelCount: globalConfig.turbolinkEegChannelCount,
        maximumDroppedSamples: globalConfig.maximumDroppedSamples,
        simulateLabjack: globalConfig.simulateLabjack,
        minimumIntertrialInterval: globalConfig.minimumIntertrialInterval,
        maximumLoopbackLatency: globalConfig.maximumLoopbackLatency,
        maximumTimingError: globalConfig.maximumTimingError,
        triggerToPulseDelay: globalConfig.triggerToPulseDelay,
        diskWarningThreshold: parseDiskThreshold(globalConfig.diskWarningThreshold),
        diskErrorThreshold: parseDiskThreshold(globalConfig.diskErrorThreshold),
        locale: globalConfig.locale,
      }
      setConfig(loadedConfig)
      setInitialConfig(loadedConfig)
      setActiveTab('eeg')
    }
  }, [isOpen, globalConfig])

  // Handle Escape key to close modal with confirmation if there are changes
  useEffect(() => {
    if (!isOpen) return

    const handleEscape = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        event.preventDefault()
        if (hasChanges()) {
          if (window.confirm('You have unsaved changes. Do you want to discard them?')) {
            onClose()
          }
        } else {
          onClose()
        }
      }
    }

    document.addEventListener('keydown', handleEscape)
    return () => document.removeEventListener('keydown', handleEscape)
  }, [isOpen, config, initialConfig])

  const handleSave = async () => {
    // Convert disk thresholds back to string format (e.g., 100 -> "100GiB")
    await globalConfig.setGlobalConfig({
      eegPort: config.eegPort,
      eegDevice: config.eegDevice,
      turbolinkSamplingFrequency: config.turbolinkSamplingFrequency,
      turbolinkEegChannelCount: config.turbolinkEegChannelCount,
      maximumDroppedSamples: config.maximumDroppedSamples,
      simulateLabjack: config.simulateLabjack,
      minimumIntertrialInterval: config.minimumIntertrialInterval,
      maximumLoopbackLatency: config.maximumLoopbackLatency,
      maximumTimingError: config.maximumTimingError,
      triggerToPulseDelay: config.triggerToPulseDelay,
      diskWarningThreshold: `${config.diskWarningThreshold}GiB`,
      diskErrorThreshold: `${config.diskErrorThreshold}GiB`,
      locale: config.locale,
    }, () => {
      console.log('Global config saved successfully')
    })
    
    onClose()
  }

  const handleClose = () => {
    if (hasChanges()) {
      if (window.confirm('You have unsaved changes. Do you want to discard them?')) {
        onClose()
      }
    } else {
      onClose()
    }
  }

  const handleFormKeyDown = (e: React.KeyboardEvent) => {
    // Unfocus field on Enter key
    if (e.key === 'Enter') {
      e.preventDefault()
      const target = e.target as HTMLElement
      if (target && target.blur) {
        target.blur()
      }
    }
  }

  const updateConfig = <K extends keyof GlobalConfigValues>(
    key: K,
    value: GlobalConfigValues[K]
  ) => {
    setConfig(prev => ({ ...prev, [key]: value }))
  }

  // Format number to show at least one decimal place, but preserve additional significant digits (up to 4 decimals)
  const formatDecimal = (value: number): string => {
    const str = value.toFixed(4)
    const trimmed = str.replace(/(\.\d*?)0+$/, '$1')
    return trimmed.endsWith('.') ? trimmed + '0' : trimmed
  }

  const InfoTooltip: React.FC<{ text: string }> = ({ text }) => {
    const [showTooltip, setShowTooltip] = useState(false)
    
    return (
      <InfoIconWrapper
        onMouseEnter={() => setShowTooltip(true)}
        onMouseLeave={() => setShowTooltip(false)}
      >
        <InfoIcon icon={faInfoCircle} />
        <Tooltip show={showTooltip}>{text}</Tooltip>
      </InfoIconWrapper>
    )
  }

  if (!isOpen) return null

  return (
    <ModalOverlay onClick={handleClose}>
      <ModalContent onClick={e => e.stopPropagation()}>
        <ModalHeader>
          <ModalTitle>Settings</ModalTitle>
          <CloseButton onClick={handleClose}>
            <FontAwesomeIcon icon={faTimes} />
          </CloseButton>
        </ModalHeader>

        <TabContainer>
          <Tab active={activeTab === 'eeg'} onClick={() => setActiveTab('eeg')} type="button">
            EEG
          </Tab>
          <Tab active={activeTab === 'labjack'} onClick={() => setActiveTab('labjack')} type="button">
            LabJack
          </Tab>
          <Tab active={activeTab === 'safety'} onClick={() => setActiveTab('safety')} type="button">
            Safety
          </Tab>
          <Tab active={activeTab === 'timing'} onClick={() => setActiveTab('timing')} type="button">
            Timing
          </Tab>
          <Tab active={activeTab === 'system'} onClick={() => setActiveTab('system')} type="button">
            System
          </Tab>
        </TabContainer>

        <Form onKeyDown={handleFormKeyDown}>
          <TabContent>
            {activeTab === 'eeg' && (
              <Section>
            
            <InputGroup>
              <Label htmlFor="eegPort">EEG Port:</Label>
              <Input
                id="eegPort"
                type="number"
                value={config.eegPort}
                onChange={(e) => updateConfig('eegPort', parseInt(e.target.value))}
              />
            </InputGroup>

            <InputGroup>
              <Label htmlFor="eegDevice">EEG Device:</Label>
              <Select
                id="eegDevice"
                value={config.eegDevice}
                onChange={(e) => updateConfig('eegDevice', e.target.value)}
              >
                <option value="neurone">NeurOne</option>
                <option value="turbolink">TurboLink</option>
              </Select>
            </InputGroup>

            {config.eegDevice === 'turbolink' && (
              <>
                <InputGroup>
                  <Label htmlFor="turbolinkSamplingFrequency">Sampling Rate (Hz):</Label>
                  <Input
                    id="turbolinkSamplingFrequency"
                    type="number"
                    value={config.turbolinkSamplingFrequency}
                    onChange={(e) => updateConfig('turbolinkSamplingFrequency', parseInt(e.target.value))}
                  />
                </InputGroup>

                <InputGroup>
                  <Label htmlFor="turbolinkEegChannelCount">Channel Count:</Label>
                  <Input
                    id="turbolinkEegChannelCount"
                    type="number"
                    value={config.turbolinkEegChannelCount}
                    onChange={(e) => updateConfig('turbolinkEegChannelCount', parseInt(e.target.value))}
                  />
                </InputGroup>
              </>
            )}

            <InputGroup>
              <LabelRow>
                <Label htmlFor="maximumDroppedSamples">Maximum Dropped Samples:</Label>
                <InfoTooltip text="Maximum consecutive dropped samples tolerated before entering error state" />
              </LabelRow>
              <Input
                id="maximumDroppedSamples"
                type="number"
                value={config.maximumDroppedSamples}
                onChange={(e) => updateConfig('maximumDroppedSamples', parseInt(e.target.value))}
              />
            </InputGroup>
              </Section>
            )}

            {activeTab === 'labjack' && (
              <Section>
            
            <InputGroup>
              <CheckboxWrapper>
                <Checkbox
                  id="simulateLabjack"
                  type="checkbox"
                  checked={config.simulateLabjack}
                  onChange={(e) => updateConfig('simulateLabjack', e.target.checked)}
                />
                <Label htmlFor="simulateLabjack">Simulate LabJack</Label>
              </CheckboxWrapper>
            </InputGroup>
              </Section>
            )}

            {activeTab === 'timing' && (
              <Section>
            
            <InputGroup>
              <LabelRow>
                <Label htmlFor="maximumLoopbackLatency">Maximum Loopback Latency (milliseconds):</Label>
                <InfoTooltip text="Exceeding this value prevents stimulation" />
              </LabelRow>
              <ValidatedInput
                value={config.maximumLoopbackLatency}
                onChange={(val) => updateConfig('maximumLoopbackLatency', val)}
                formatValue={(val) => formatDecimal(val * 1000)}
                parseValue={(str) => parseFloat(str) / 1000}
                min={0}
                step={0.1}
              />
            </InputGroup>

            <InputGroup>
              <LabelRow>
                <Label htmlFor="maximumTimingError">Maximum Timing Error (milliseconds):</Label>
                <InfoTooltip text="Maximum timing error for triggering" />
              </LabelRow>
              <ValidatedInput
                value={config.maximumTimingError}
                onChange={(val) => updateConfig('maximumTimingError', val)}
                formatValue={(val) => formatDecimal(val * 1000)}
                parseValue={(str) => parseFloat(str) / 1000}
                min={0}
                step={0.1}
              />
            </InputGroup>

            <InputGroup>
              <LabelRow>
                <Label htmlFor="triggerToPulseDelay">Trigger to Pulse Delay (milliseconds):</Label>
                <InfoTooltip text="Delay between trigger and pulse" />
              </LabelRow>
              <ValidatedInput
                value={config.triggerToPulseDelay}
                onChange={(val) => updateConfig('triggerToPulseDelay', val)}
                formatValue={(val) => formatDecimal(val * 1000)}
                parseValue={(str) => parseFloat(str) / 1000}
                min={0}
                step={0.1}
              />
            </InputGroup>
              </Section>
            )}

            {activeTab === 'safety' && (
              <Section>
            
            <InputGroup>
              <LabelRow>
                <Label htmlFor="minimumIntertrialInterval">Minimum Intertrial Interval (seconds):</Label>
                <InfoTooltip text="Minimum time between consecutive TMS trials (e.g., 2.5, 2.0, or 2)" />
              </LabelRow>
              <ValidatedInput
                value={config.minimumIntertrialInterval}
                onChange={(val) => updateConfig('minimumIntertrialInterval', val)}
                formatValue={formatDecimal}
                min={0}
                step={0.1}
              />
            </InputGroup>
              </Section>
            )}

            {activeTab === 'system' && (
              <Section>
            
            <InputGroup>
              <Label htmlFor="locale">Locale (date/time format):</Label>
              <Select
                id="locale"
                value={config.locale}
                onChange={(e) => updateConfig('locale', e.target.value)}
              >
                {COMMON_LOCALES.map((loc) => (
                  <option key={loc.value} value={loc.value}>
                    {loc.label}
                  </option>
                ))}
              </Select>
            </InputGroup>

            <InputGroup>
              <Label htmlFor="diskWarningThreshold">Disk Warning Threshold (GiB):</Label>
              <Input
                id="diskWarningThreshold"
                type="number"
                value={config.diskWarningThreshold}
                onChange={(e) => updateConfig('diskWarningThreshold', parseInt(e.target.value))}
              />
            </InputGroup>

            <InputGroup>
              <Label htmlFor="diskErrorThreshold">Disk Error Threshold (GiB):</Label>
              <Input
                id="diskErrorThreshold"
                type="number"
                value={config.diskErrorThreshold}
                onChange={(e) => updateConfig('diskErrorThreshold', parseInt(e.target.value))}
              />
            </InputGroup>
              </Section>
            )}
          </TabContent>

          <ButtonGroup>
            <Button type="button" variant="secondary" onClick={handleClose}>
              Cancel
            </Button>
            <Button type="button" variant="primary" onClick={handleSave}>
              Save
            </Button>
          </ButtonGroup>
        </Form>
      </ModalContent>
    </ModalOverlay>
  )
}
