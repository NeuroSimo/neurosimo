import React, { useEffect } from 'react'
import styled from 'styled-components'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'
import { faTimes, faInfoCircle } from '@fortawesome/free-solid-svg-icons'
import { RecordingInfo } from 'ros/recording'
import { formatDateTime } from 'utils/utils'

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
  display: flex;
  align-items: center;
  gap: 8px;
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

const InfoGrid = styled.div`
  display: grid;
  grid-template-columns: 220px 1fr;
  gap: 12px 16px;
  align-items: baseline;
`

const InfoLabel = styled.div`
  font-weight: 600;
  color: #555;
  font-size: 14px;
  text-transform: uppercase;
  letter-spacing: 0.5px;
  padding-top: 2px;
`

const InfoValue = styled.div`
  color: #333;
  word-break: break-word;
  padding-top: 2px;
`

const SectionHeader = styled.div`
  font-weight: 600;
  color: #444;
  font-size: 16px;
  margin: 20px 0 12px 0;
  border-bottom: 1px solid #e0e0e0;
  padding-bottom: 4px;
`

interface RecordingInfoModalProps {
  isOpen: boolean
  onClose: () => void
  recordingInfo: RecordingInfo | null
  recordingName: string
}

export const RecordingInfoModal: React.FC<RecordingInfoModalProps> = ({
  isOpen,
  onClose,
  recordingInfo,
  recordingName,
}) => {
  // Handle Escape key to close modal
  useEffect(() => {
    if (!isOpen) return

    const handleEscape = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        onClose()
      }
    }

    document.addEventListener('keydown', handleEscape)
    return () => document.removeEventListener('keydown', handleEscape)
  }, [isOpen, onClose])

  if (!isOpen || !recordingInfo) return null

  const formatFingerprintHex = (value?: number | null): string => {
    if (!value) {
      return 'Not available'
    }
    const full = value.toString(16).toUpperCase().padStart(16, '0')
    return full
  }

  return (
    <ModalOverlay onClick={onClose}>
      <ModalContent onClick={e => e.stopPropagation()}>
        <ModalHeader>
          <ModalTitle>
            <FontAwesomeIcon icon={faInfoCircle} />
            Recording Information - {recordingName}
          </ModalTitle>
          <CloseButton onClick={onClose}>
            <FontAwesomeIcon icon={faTimes} />
          </CloseButton>
        </ModalHeader>

        <SectionHeader>Project & Protocol</SectionHeader>
        <InfoGrid>
          <InfoLabel>Project Name</InfoLabel>
          <InfoValue>{recordingInfo.project_name || 'Not specified'}</InfoValue>

          <InfoLabel>Protocol Name</InfoLabel>
          <InfoValue>{recordingInfo.protocol_name || 'Not specified'}</InfoValue>

          <InfoLabel>Protocol Filename</InfoLabel>
          <InfoValue>{recordingInfo.protocol_filename || 'Not specified'}</InfoValue>
        </InfoGrid>

        <SectionHeader>Timing</SectionHeader>
        <InfoGrid>
          <InfoLabel>Start Time</InfoLabel>
          <InfoValue>{formatDateTime(recordingInfo.start_time, 'en-US')}</InfoValue>

          <InfoLabel>End Time</InfoLabel>
          <InfoValue>{recordingInfo.end_time ? formatDateTime(recordingInfo.end_time, 'en-US') : 'Not available'}</InfoValue>

          <InfoLabel>Duration</InfoLabel>
          <InfoValue>{Math.round(recordingInfo.duration)} seconds</InfoValue>
        </InfoGrid>

        <SectionHeader>Version & Git</SectionHeader>
        <InfoGrid>
          <InfoLabel>Version</InfoLabel>
          <InfoValue>{recordingInfo.version || 'Not available'}</InfoValue>

          <InfoLabel>Git Commit</InfoLabel>
          <InfoValue style={{ fontFamily: 'monospace', fontSize: '13px' }}>
            {recordingInfo.git_commit || 'Not available'}
          </InfoValue>

          <InfoLabel>Git State</InfoLabel>
          <InfoValue>{recordingInfo.git_state || 'Not available'}</InfoValue>
        </InfoGrid>

        <SectionHeader>Data Source Details</SectionHeader>
        <InfoGrid>
          <InfoLabel>Data Source</InfoLabel>
          <InfoValue>{recordingInfo.data_source}</InfoValue>

          {recordingInfo.data_source === 'simulator' && (
            <>
              <InfoLabel>Simulator Dataset</InfoLabel>
              <InfoValue>{recordingInfo.simulator_dataset_filename || 'Not specified'}</InfoValue>

              <InfoLabel>Simulator Start Time</InfoLabel>
              <InfoValue>{recordingInfo.simulator_start_time || 'Not specified'}</InfoValue>
            </>
          )}

          {recordingInfo.data_source === 'recording' && (
            <>
              <InfoLabel>Replay Bag ID</InfoLabel>
              <InfoValue>{recordingInfo.replay_bag_id || 'Not specified'}</InfoValue>

              <InfoLabel>Replay Preprocessed</InfoLabel>
              <InfoValue>{recordingInfo.replay_play_preprocessed ? 'Yes' : 'No'}</InfoValue>
            </>
          )}
        </InfoGrid>

        <SectionHeader>Fingerprints</SectionHeader>
        <InfoGrid>
          <InfoLabel>Data Source Fingerprint</InfoLabel>
          <InfoValue style={{ fontFamily: 'monospace', fontSize: '13px' }}>
            {formatFingerprintHex(recordingInfo.data_source_fingerprint)}
          </InfoValue>

          <InfoLabel>Preprocessor Fingerprint</InfoLabel>
          <InfoValue style={{ fontFamily: 'monospace', fontSize: '13px' }}>
            {formatFingerprintHex(recordingInfo.preprocessor_fingerprint)}
          </InfoValue>

          <InfoLabel>Decision Fingerprint</InfoLabel>
          <InfoValue style={{ fontFamily: 'monospace', fontSize: '13px' }}>
            {formatFingerprintHex(recordingInfo.decision_fingerprint)}
          </InfoValue>
        </InfoGrid>

        <SectionHeader>Export Status</SectionHeader>
        <InfoGrid>
          <InfoLabel>Exported</InfoLabel>
          <InfoValue>{recordingInfo.exported ? 'Yes' : 'No'}</InfoValue>

          <InfoLabel>Export Directory</InfoLabel>
          <InfoValue style={{ fontFamily: 'monospace', fontSize: '13px' }}>
            {recordingInfo.export_directory || 'Not available'}
          </InfoValue>
        </InfoGrid>
      </ModalContent>
    </ModalOverlay>
  )
}