import React, { useState } from 'react'
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
  z-index: 1000;
`

const ModalContent = styled.div`
  background: white;
  border-radius: 8px;
  padding: 20px;
  max-width: 400px;
  width: 90%;
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

const CheckboxGroup = styled.div`
  margin-bottom: 16px;
`

const CheckboxLabel = styled.label`
  display: flex;
  align-items: center;
  margin-bottom: 8px;
  cursor: pointer;
`

const Checkbox = styled.input`
  margin-right: 8px;
`

const ButtonGroup = styled.div`
  display: flex;
  justify-content: flex-end;
  gap: 12px;
  margin-top: 20px;
`

const Button = styled.button<{ primary?: boolean }>`
  padding: 8px 16px;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  font-weight: 500;
  transition: background-color 0.2s;

  ${props => props.primary ? `
    background-color: #007bff;
    color: white;
    &:hover {
      background-color: #0056b3;
    }
  ` : `
    background-color: #f8f9fa;
    color: #333;
    border: 1px solid #ddd;
    &:hover {
      background-color: #e9ecef;
    }
  `}
`

export enum ExportDataType {
  RAW_EEG = 0,
  ENRICHED_EEG = 1,
  PREPROCESSED_EEG = 2,
  STIMULATION_DECISIONS = 3,
  DECIDER_LOGS = 4,
  PREPROCESSOR_LOGS = 5,
  PRESENTER_LOGS = 6,
  SENSORY_STIMULI = 7,
}

const DATA_TYPE_LABELS: Record<ExportDataType, string> = {
  [ExportDataType.RAW_EEG]: 'Raw EEG',
  [ExportDataType.ENRICHED_EEG]: 'Enriched EEG',
  [ExportDataType.PREPROCESSED_EEG]: 'Preprocessed EEG',
  [ExportDataType.STIMULATION_DECISIONS]: 'Stimulation Decisions',
  [ExportDataType.DECIDER_LOGS]: 'Decider Logs',
  [ExportDataType.PREPROCESSOR_LOGS]: 'Preprocessor Logs',
  [ExportDataType.PRESENTER_LOGS]: 'Presenter Logs',
  [ExportDataType.SENSORY_STIMULI]: 'Sensory Stimuli',
}

interface ExportModalProps {
  isOpen: boolean
  onClose: () => void
  onExport: (selectedTypes: ExportDataType[]) => void
  recordingName: string
}

export const ExportModal: React.FC<ExportModalProps> = ({
  isOpen,
  onClose,
  onExport,
  recordingName,
}) => {
  const [selectedTypes, setSelectedTypes] = useState<Set<ExportDataType>>(new Set())

  const handleTypeChange = (type: ExportDataType, checked: boolean) => {
    const newSelected = new Set(selectedTypes)
    if (checked) {
      newSelected.add(type)
    } else {
      newSelected.delete(type)
    }
    setSelectedTypes(newSelected)
  }

  const handleExport = () => {
    onExport(Array.from(selectedTypes))
    setSelectedTypes(new Set())
    onClose()
  }

  const handleCancel = () => {
    setSelectedTypes(new Set())
    onClose()
  }

  if (!isOpen) return null

  return (
    <ModalOverlay onClick={onClose}>
      <ModalContent onClick={e => e.stopPropagation()}>
        <ModalHeader>
          <ModalTitle>Export Data - {recordingName}</ModalTitle>
          <CloseButton onClick={onClose}>
            <FontAwesomeIcon icon={faTimes} />
          </CloseButton>
        </ModalHeader>

        <CheckboxGroup>
          {Object.entries(DATA_TYPE_LABELS).map(([key, label]) => {
            const type = parseInt(key) as ExportDataType
            return (
              <CheckboxLabel key={type}>
                <Checkbox
                  type="checkbox"
                  checked={selectedTypes.has(type)}
                  onChange={(e) => handleTypeChange(type, e.target.checked)}
                />
                {label}
              </CheckboxLabel>
            )
          })}
        </CheckboxGroup>

        <ButtonGroup>
          <Button onClick={handleCancel}>Cancel</Button>
          <Button primary onClick={handleExport} disabled={selectedTypes.size === 0}>
            Export
          </Button>
        </ButtonGroup>
      </ModalContent>
    </ModalOverlay>
  )
}