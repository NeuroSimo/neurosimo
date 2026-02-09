import React, { useState, useEffect } from 'react'
import styled from 'styled-components'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'
import { faTimes } from '@fortawesome/free-solid-svg-icons'

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
  max-width: 600px;
  width: 90%;
  max-height: 80vh;
  overflow-y: auto;
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

const Form = styled.form`
  display: flex;
  flex-direction: column;
  gap: 20px;
`

const Section = styled.div`
  display: flex;
  flex-direction: column;
  gap: 12px;
`

const SectionTitle = styled.h4`
  margin: 0;
  color: #555;
  font-size: 16px;
  border-bottom: 1px solid #e0e0e0;
  padding-bottom: 8px;
`

const InputGroup = styled.div`
  display: flex;
  flex-direction: column;
  gap: 6px;
`

const Label = styled.label`
  font-weight: 500;
  color: #555;
  font-size: 13px;
`

const HelpText = styled.div`
  font-size: 12px;
  color: #777;
  font-style: italic;
  margin-top: -4px;
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
  numOfToleratedDroppedSamples: number
  
  // LabJack Configuration
  simulateLabjack: boolean
  
  // Safety Configuration
  minimumIntertrialInterval: number
  maximumLoopbackLatency: number
  maximumTimingError: number
  
  // Disk Space Monitoring Configuration
  diskWarningThreshold: string
  diskErrorThreshold: string
}

const defaultConfig: GlobalConfigValues = {
  // EEG Configuration
  eegPort: 50000,
  eegDevice: 'neurone',
  turbolinkSamplingFrequency: 5000,
  turbolinkEegChannelCount: 64,
  numOfToleratedDroppedSamples: 2,
  
  // LabJack Configuration
  simulateLabjack: false,
  
  // Safety Configuration
  minimumIntertrialInterval: 2.0,
  maximumLoopbackLatency: 0.005,
  maximumTimingError: 0.0,
  
  // Disk Space Monitoring Configuration
  diskWarningThreshold: '100GiB',
  diskErrorThreshold: '50GiB',
}

export const GlobalConfigModal: React.FC<GlobalConfigModalProps> = ({
  isOpen,
  onClose,
}) => {
  const [config, setConfig] = useState<GlobalConfigValues>(defaultConfig)

  // Load config when modal opens (placeholder - will be connected to backend later)
  useEffect(() => {
    if (isOpen) {
      // TODO: Load actual config from backend
      setConfig(defaultConfig)
    }
  }, [isOpen])

  // Handle Escape key to close modal
  useEffect(() => {
    if (!isOpen) return

    const handleEscape = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        handleClose()
      }
    }

    document.addEventListener('keydown', handleEscape)
    return () => document.removeEventListener('keydown', handleEscape)
  }, [isOpen])

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault()
    
    // TODO: Save config to backend
    console.log('Saving global config:', config)
    
    onClose()
  }

  const handleClose = () => {
    onClose()
  }

  const updateConfig = <K extends keyof GlobalConfigValues>(
    key: K,
    value: GlobalConfigValues[K]
  ) => {
    setConfig(prev => ({ ...prev, [key]: value }))
  }

  if (!isOpen) return null

  return (
    <ModalOverlay onClick={handleClose}>
      <ModalContent onClick={e => e.stopPropagation()}>
        <ModalHeader>
          <ModalTitle>Global Configuration</ModalTitle>
          <CloseButton onClick={handleClose}>
            <FontAwesomeIcon icon={faTimes} />
          </CloseButton>
        </ModalHeader>

        <Form onSubmit={handleSubmit}>
          {/* EEG Configuration */}
          <Section>
            <SectionTitle>EEG Configuration</SectionTitle>
            
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
              <HelpText>Supported values: &apos;neurone&apos;, &apos;turbolink&apos;</HelpText>
            </InputGroup>

            <InputGroup>
              <Label htmlFor="turbolinkSamplingFrequency">TurboLink Sampling Frequency (Hz):</Label>
              <Input
                id="turbolinkSamplingFrequency"
                type="number"
                value={config.turbolinkSamplingFrequency}
                onChange={(e) => updateConfig('turbolinkSamplingFrequency', parseInt(e.target.value))}
              />
              <HelpText>Ignored for NeurOne devices (gets automatically from device)</HelpText>
            </InputGroup>

            <InputGroup>
              <Label htmlFor="turbolinkEegChannelCount">TurboLink EEG Channel Count:</Label>
              <Input
                id="turbolinkEegChannelCount"
                type="number"
                value={config.turbolinkEegChannelCount}
                onChange={(e) => updateConfig('turbolinkEegChannelCount', parseInt(e.target.value))}
              />
              <HelpText>Ignored for NeurOne devices (gets automatically from device)</HelpText>
            </InputGroup>

            <InputGroup>
              <Label htmlFor="numOfToleratedDroppedSamples">Number of Tolerated Dropped Samples:</Label>
              <Input
                id="numOfToleratedDroppedSamples"
                type="number"
                value={config.numOfToleratedDroppedSamples}
                onChange={(e) => updateConfig('numOfToleratedDroppedSamples', parseInt(e.target.value))}
              />
              <HelpText>Consecutive dropped samples tolerated before entering error state</HelpText>
            </InputGroup>
          </Section>

          {/* LabJack Configuration */}
          <Section>
            <SectionTitle>LabJack Configuration</SectionTitle>
            
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

          {/* Safety Configuration */}
          <Section>
            <SectionTitle>Safety Configuration</SectionTitle>
            
            <InputGroup>
              <Label htmlFor="minimumIntertrialInterval">Minimum Intertrial Interval (seconds):</Label>
              <Input
                id="minimumIntertrialInterval"
                type="number"
                step="0.1"
                value={config.minimumIntertrialInterval}
                onChange={(e) => updateConfig('minimumIntertrialInterval', parseFloat(e.target.value))}
              />
              <HelpText>Minimum time between consecutive TMS trials (e.g., 2.5, 2.0, or 2)</HelpText>
            </InputGroup>

            <InputGroup>
              <Label htmlFor="maximumLoopbackLatency">Maximum Loopback Latency (seconds):</Label>
              <Input
                id="maximumLoopbackLatency"
                type="number"
                step="0.001"
                value={config.maximumLoopbackLatency}
                onChange={(e) => updateConfig('maximumLoopbackLatency', parseFloat(e.target.value))}
              />
              <HelpText>Exceeding this value prevents stimulation</HelpText>
            </InputGroup>

            <InputGroup>
              <Label htmlFor="maximumTimingError">Maximum Timing Error (seconds):</Label>
              <Input
                id="maximumTimingError"
                type="number"
                step="0.001"
                value={config.maximumTimingError}
                onChange={(e) => updateConfig('maximumTimingError', parseFloat(e.target.value))}
              />
              <HelpText>Maximum timing error for triggering</HelpText>
            </InputGroup>
          </Section>

          {/* Disk Space Monitoring Configuration */}
          <Section>
            <SectionTitle>Disk Space Monitoring</SectionTitle>
            
            <InputGroup>
              <Label htmlFor="diskWarningThreshold">Warning Threshold:</Label>
              <Input
                id="diskWarningThreshold"
                type="text"
                value={config.diskWarningThreshold}
                onChange={(e) => updateConfig('diskWarningThreshold', e.target.value)}
              />
              <HelpText>Example: 100GiB, 50GB</HelpText>
            </InputGroup>

            <InputGroup>
              <Label htmlFor="diskErrorThreshold">Error Threshold:</Label>
              <Input
                id="diskErrorThreshold"
                type="text"
                value={config.diskErrorThreshold}
                onChange={(e) => updateConfig('diskErrorThreshold', e.target.value)}
              />
              <HelpText>Example: 50GiB, 25GB</HelpText>
            </InputGroup>
          </Section>

          <ButtonGroup>
            <Button type="button" variant="secondary" onClick={handleClose}>
              Cancel
            </Button>
            <Button type="submit" variant="primary">
              Save
            </Button>
          </ButtonGroup>
        </Form>
      </ModalContent>
    </ModalOverlay>
  )
}
