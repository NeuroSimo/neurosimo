import React from 'react'
import styled from 'styled-components'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'
import { faTimes } from '@fortawesome/free-solid-svg-icons'
import { ProtocolInfo, PROTOCOL_ELEMENT_TYPE } from 'ros/experiment'
import { palette } from 'styles/General'

const ModalOverlay = styled.div`
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: rgba(0, 0, 0, 0.6);
  display: flex;
  justify-content: center;
  align-items: center;
  z-index: 1100;
`

const ModalContent = styled.div`
  background: ${palette.surface};
  color: ${palette.text};
  color-scheme: dark;
  border: 1px solid ${palette.borderStrong};
  border-radius: 8px;
  padding: 20px;
  max-width: 600px;
  width: 90%;
  max-height: 80vh;
  overflow-y: auto;
  box-shadow: 0 8px 24px rgba(0, 0, 0, 0.5);
`

const ModalHeader = styled.div`
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 16px;
  border-bottom: 1px solid ${palette.border};
  padding-bottom: 12px;
`

const ModalTitle = styled.h3`
  margin: 0;
  color: ${palette.text};
`

const CloseButton = styled.button`
  background: none;
  border: none;
  font-size: 18px;
  cursor: pointer;
  color: ${palette.icon};
  &:hover {
    color: ${palette.iconHover};
  }
`

const InfoSection = styled.div`
  display: flex;
  flex-direction: column;
  gap: 12px;
`

const InfoRow = styled.div`
  display: flex;
  gap: 8px;
`

const InfoLabel = styled.span`
  font-weight: 500;
  color: ${palette.textSecondary};
  min-width: 160px;
`

const InfoValue = styled.span`
  color: ${palette.text};
  flex: 1;
`

const ElementsSection = styled.div`
  margin-top: 16px;
`

const ElementsSectionTitle = styled.h4`
  margin: 0 0 12px 0;
  color: ${palette.text};
  font-size: 16px;
`

type ElementVariant = 'stage' | 'rest' | 'task'

const elementCardStyles: Record<
  ElementVariant,
  { bg: string; border: string }
> = {
  stage: { bg: 'rgba(74, 138, 212, 0.12)', border: palette.blue },
  rest: { bg: 'rgba(229, 149, 74, 0.12)', border: palette.orange },
  task: { bg: 'rgba(79, 191, 139, 0.12)', border: palette.green },
}

const ElementCard = styled.div<{ $variant: ElementVariant }>`
  padding: 12px;
  margin-bottom: 8px;
  background-color: ${props => elementCardStyles[props.$variant].bg};
  border-left: 4px solid ${props => elementCardStyles[props.$variant].border};
  border-radius: 4px;
`

const ElementHeader = styled.div`
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 4px;
`

const elementTypeColors: Record<ElementVariant, string> = {
  stage: '#7fb0e8',
  rest: palette.orange,
  task: palette.green,
}

const ElementType = styled.span<{ $variant: ElementVariant }>`
  font-weight: 600;
  color: ${props => elementTypeColors[props.$variant]};
  text-transform: uppercase;
  font-size: 12px;
`

const ElementName = styled.span`
  font-weight: 600;
  color: ${palette.text};
  font-size: 14px;
`

const ElementDetails = styled.div`
  font-size: 13px;
  color: ${palette.textSecondary};
  margin-top: 4px;
`

const ElementNotes = styled.div`
  font-size: 12px;
  color: ${palette.textMuted};
  font-style: italic;
  margin-top: 6px;
`

interface ProtocolInfoModalProps {
  isOpen: boolean
  onClose: () => void
  protocolInfo: ProtocolInfo | null
}

export const ProtocolInfoModal: React.FC<ProtocolInfoModalProps> = ({
  isOpen,
  onClose,
  protocolInfo,
}) => {
  if (!isOpen || !protocolInfo) return null

  const totalStages = protocolInfo.elements.filter(e => e.type === PROTOCOL_ELEMENT_TYPE.STAGE).length
  const totalTrials = protocolInfo.elements
    .filter(e => e.type === PROTOCOL_ELEMENT_TYPE.STAGE)
    .reduce((sum, e) => sum + e.stage.trials, 0)
  const totalRests = protocolInfo.elements.filter(e => e.type === PROTOCOL_ELEMENT_TYPE.REST).length
  const totalTasks = protocolInfo.elements.filter(e => e.type === PROTOCOL_ELEMENT_TYPE.TASK).length

  return (
    <ModalOverlay onClick={onClose}>
      <ModalContent onClick={e => e.stopPropagation()}>
        <ModalHeader>
          <ModalTitle>{protocolInfo.name}</ModalTitle>
          <CloseButton onClick={onClose}>
            <FontAwesomeIcon icon={faTimes} />
          </CloseButton>
        </ModalHeader>

        <InfoSection>
          <InfoRow>
            <InfoLabel>Filename:</InfoLabel>
            <InfoValue>{protocolInfo.yaml_filename}</InfoValue>
          </InfoRow>
          {protocolInfo.description && (
            <InfoRow>
              <InfoLabel>Description:</InfoLabel>
              <InfoValue>{protocolInfo.description}</InfoValue>
            </InfoRow>
          )}
          <InfoRow>
            <InfoLabel>Trials:</InfoLabel>
            <InfoValue>{totalTrials}</InfoValue>
          </InfoRow>
          <InfoRow>
            <InfoLabel>Minimum trial interval:</InfoLabel>
            <InfoValue>{protocolInfo.minimum_trial_interval} s</InfoValue>
          </InfoRow>
          <InfoRow>
            <InfoLabel>Repeat failed trials:</InfoLabel>
            <InfoValue>{protocolInfo.repeat_failed_trials ? 'Yes' : 'No'}</InfoValue>
          </InfoRow>
        </InfoSection>

        <ElementsSection>
          <ElementsSectionTitle>Steps</ElementsSectionTitle>
          {protocolInfo.elements.map((element, index) => {
            switch (element.type) {
              case PROTOCOL_ELEMENT_TYPE.STAGE:
                return (
                  <ElementCard key={index} $variant="stage">
                    <ElementHeader>
                      <ElementType $variant="stage">Stage</ElementType>
                      <ElementName>{element.stage.name}</ElementName>
                    </ElementHeader>
                    <ElementDetails>
                      Trials: {element.stage.trials}
                    </ElementDetails>
                    {element.stage.notes && (
                      <ElementNotes>{element.stage.notes}</ElementNotes>
                    )}
                  </ElementCard>
                )
              case PROTOCOL_ELEMENT_TYPE.REST:
                return (
                  <ElementCard key={index} $variant="rest">
                    <ElementHeader>
                      <ElementType $variant="rest">Rest</ElementType>
                    </ElementHeader>
                    {element.rest.notes && (
                      <ElementNotes>{element.rest.notes}</ElementNotes>
                    )}
                  </ElementCard>
                )
              case PROTOCOL_ELEMENT_TYPE.TASK:
                return (
                  <ElementCard key={index} $variant="task">
                    <ElementHeader>
                      <ElementType $variant="task">Task</ElementType>
                      <ElementName>{element.task.name}</ElementName>
                    </ElementHeader>
                    {element.task.notes && (
                      <ElementNotes>{element.task.notes}</ElementNotes>
                    )}
                  </ElementCard>
                )
              default:
                return null
            }
          })}
        </ElementsSection>
      </ModalContent>
    </ModalOverlay>
  )
}
