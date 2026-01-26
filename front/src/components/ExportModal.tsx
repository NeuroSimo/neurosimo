import React, { useState, useEffect, useRef } from 'react'
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

const GroupHeader = styled.div`
  font-weight: 600;
  color: #555;
  font-size: 14px;
  margin: 16px 0 8px 0;
  text-transform: uppercase;
  letter-spacing: 0.5px;
`

const CheckboxLabel = styled.label<{ disabled?: boolean }>`
  display: flex;
  align-items: center;
  margin-bottom: 8px;
  cursor: ${props => props.disabled ? 'default' : 'pointer'};
  margin-left: 12px;
  color: ${props => props.disabled ? '#999' : 'inherit'};
`

const Checkbox = styled.input`
  margin-right: 8px;
`

const SelectAllContainer = styled.div`
  margin-bottom: 16px;
  padding-bottom: 12px;
  border-bottom: 1px solid #e0e0e0;
`

const SelectAllLabel = styled.label`
  display: flex;
  align-items: center;
  cursor: pointer;
  font-weight: 500;
  color: #666;
  font-size: 14px;
`

const SelectAllCheckbox = styled.input`
  margin-right: 10px;
  transform: scale(1.3);
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

  &:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }

  ${props => props.primary ? `
    background-color: #007bff;
    color: white;
    &:hover:not(:disabled) {
      background-color: #0056b3;
    }
  ` : `
    background-color: #f8f9fa;
    color: #333;
    border: 1px solid #ddd;
    &:hover:not(:disabled) {
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
  [ExportDataType.RAW_EEG]: 'Raw',
  [ExportDataType.ENRICHED_EEG]: 'Enriched',
  [ExportDataType.PREPROCESSED_EEG]: 'Preprocessed',
  [ExportDataType.STIMULATION_DECISIONS]: 'Pulses',
  [ExportDataType.DECIDER_LOGS]: 'Decider',
  [ExportDataType.PREPROCESSOR_LOGS]: 'Preprocessor',
  [ExportDataType.PRESENTER_LOGS]: 'Presenter',
  [ExportDataType.SENSORY_STIMULI]: 'Sensory stimuli',
}

const EXPORT_GROUPS = [
  {
    name: 'EEG',
    types: [ExportDataType.RAW_EEG, ExportDataType.ENRICHED_EEG, ExportDataType.PREPROCESSED_EEG]
  },
  {
    name: 'Decisions',
    types: [ExportDataType.STIMULATION_DECISIONS, ExportDataType.SENSORY_STIMULI]
  },
  {
    name: 'Logs',
    types: [ExportDataType.PREPROCESSOR_LOGS, ExportDataType.DECIDER_LOGS, ExportDataType.PRESENTER_LOGS]
  }
]

const STORAGE_KEY = 'exportModalSelectedTypes'

interface ExportModalProps {
  isOpen: boolean
  onClose: () => void
  onExport: (selectedTypes: ExportDataType[]) => void
  recordingName: string
  preprocessorEnabled?: boolean
  deciderEnabled?: boolean
  presenterEnabled?: boolean
}

export const ExportModal: React.FC<ExportModalProps> = ({
  isOpen,
  onClose,
  onExport,
  recordingName,
  preprocessorEnabled = true,
  deciderEnabled = true,
  presenterEnabled = true,
}) => {
  const [selectedTypes, setSelectedTypes] = useState<Set<ExportDataType>>(new Set())
  const selectAllCheckboxRef = useRef<HTMLInputElement>(null)

  // Load previously selected types from localStorage on mount
  useEffect(() => {
    try {
      const stored = localStorage.getItem(STORAGE_KEY)
      if (stored) {
        const parsedTypes: ExportDataType[] = JSON.parse(stored)
        setSelectedTypes(new Set(parsedTypes))
      }
    } catch (error) {
      console.warn('Failed to load export types from localStorage:', error)
    }
  }, [])

  // Remove disabled types from selection when component props change
  useEffect(() => {
    setSelectedTypes(currentSelected => {
      const newSelected = new Set(currentSelected)
      const disabledTypes = [
        ...(preprocessorEnabled ? [] : [ExportDataType.PREPROCESSED_EEG, ExportDataType.PREPROCESSOR_LOGS]),
        ...(deciderEnabled ? [] : [ExportDataType.DECIDER_LOGS]),
        ...(presenterEnabled ? [] : [ExportDataType.SENSORY_STIMULI, ExportDataType.PRESENTER_LOGS])
      ]
      disabledTypes.forEach(type => newSelected.delete(type))
      return newSelected
    })
  }, [preprocessorEnabled, deciderEnabled, presenterEnabled])

  // Save selected types to localStorage whenever they change
  useEffect(() => {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(Array.from(selectedTypes)))
    } catch (error) {
      console.warn('Failed to save export types to localStorage:', error)
    }
  }, [selectedTypes])

  // Manage the indeterminate state of the select all checkbox
  useEffect(() => {
    const checkbox = selectAllCheckboxRef.current
    if (checkbox) {
      const allTypes = Object.keys(DATA_TYPE_LABELS).map(key => parseInt(key) as ExportDataType)
      const enabledTypes = allTypes.filter(type =>
        !((type === ExportDataType.PREPROCESSED_EEG && !preprocessorEnabled) ||
          (type === ExportDataType.SENSORY_STIMULI && !presenterEnabled) ||
          (type === ExportDataType.DECIDER_LOGS && !deciderEnabled) ||
          (type === ExportDataType.PREPROCESSOR_LOGS && !preprocessorEnabled) ||
          (type === ExportDataType.PRESENTER_LOGS && !presenterEnabled))
      )
      const selectedEnabledCount = enabledTypes.filter(type => selectedTypes.has(type)).length

      if (selectedEnabledCount === 0) {
        checkbox.checked = false
        checkbox.indeterminate = false
      } else if (selectedEnabledCount === enabledTypes.length) {
        checkbox.checked = true
        checkbox.indeterminate = false
      } else {
        checkbox.checked = false
        checkbox.indeterminate = true
      }
    }
  }, [selectedTypes, preprocessorEnabled, deciderEnabled, presenterEnabled])

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
    onClose()
  }

  const handleCancel = () => {
    onClose()
  }

  const handleSelectAll = () => {
    const allTypes = Object.keys(DATA_TYPE_LABELS).map(key => parseInt(key) as ExportDataType)
    const enabledTypes = allTypes.filter(type =>
      !((type === ExportDataType.PREPROCESSED_EEG && !preprocessorEnabled) ||
        (type === ExportDataType.SENSORY_STIMULI && !presenterEnabled) ||
        (type === ExportDataType.DECIDER_LOGS && !deciderEnabled) ||
        (type === ExportDataType.PREPROCESSOR_LOGS && !preprocessorEnabled) ||
        (type === ExportDataType.PRESENTER_LOGS && !presenterEnabled))
    )
    const selectedEnabledCount = enabledTypes.filter(type => selectedTypes.has(type)).length

    if (selectedEnabledCount === enabledTypes.length) {
      // All enabled selected, deselect all
      setSelectedTypes(new Set())
    } else {
      // None or partially selected, select all enabled
      setSelectedTypes(new Set(enabledTypes))
    }
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

        <SelectAllContainer>
          <SelectAllLabel>
            <SelectAllCheckbox
              ref={selectAllCheckboxRef}
              type="checkbox"
              onChange={handleSelectAll}
            />
            {Object.keys(DATA_TYPE_LABELS).every(key => selectedTypes.has(parseInt(key) as ExportDataType))
              ? 'Clear all'
              : 'Select all'}
          </SelectAllLabel>
        </SelectAllContainer>

        <CheckboxGroup>
          {EXPORT_GROUPS.map((group) => (
            <div key={group.name}>
              <GroupHeader>{group.name}</GroupHeader>
              {group.types.map((type) => {
                const isDisabled = (type === ExportDataType.PREPROCESSED_EEG && !preprocessorEnabled) ||
                                 (type === ExportDataType.SENSORY_STIMULI && !presenterEnabled) ||
                                 (type === ExportDataType.DECIDER_LOGS && !deciderEnabled) ||
                                 (type === ExportDataType.PREPROCESSOR_LOGS && !preprocessorEnabled) ||
                                 (type === ExportDataType.PRESENTER_LOGS && !presenterEnabled)
                return (
                  <CheckboxLabel key={type} disabled={isDisabled}>
                    <Checkbox
                      type="checkbox"
                      checked={selectedTypes.has(type) && !isDisabled}
                      onChange={(e) => handleTypeChange(type, e.target.checked)}
                      disabled={isDisabled}
                    />
                    {DATA_TYPE_LABELS[type]}
                  </CheckboxLabel>
                )
              })}
            </div>
          ))}
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