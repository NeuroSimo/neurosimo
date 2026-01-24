import React, { useContext } from 'react'
import styled from 'styled-components'

import { StyledPanel, SmallerTitle, ConfigRow, ConfigLabel, CONFIG_PANEL_WIDTH } from 'styles/General'
import { useParameters } from 'providers/ParameterProvider'
import { useSession, SessionStage } from 'providers/SessionProvider'
import { CommittableTextInput } from './CommittableTextInput'
import { CommittableNumericInput } from './CommittableNumericInput'

const Container = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH}px;
  position: relative;
  margin-top: 0;
  margin-left: 0;
  left: 0;
`

export const SubjectPanel: React.FC = () => {
  const { metadata, setSubjectId, setNotes } = useParameters()
  const { sessionState } = useSession()

  const isSessionRunning = sessionState.stage !== SessionStage.STOPPED

  const handleSubjectIdCommit = (value: string) => {
    setSubjectId(value, () => {
      console.log('Subject ID set to ' + value)
    })
  }

  const handleNotesCommit = (value: string) => {
    setNotes(value, () => {
      console.log('Notes set to ' + value)
    })
  }

  return (
    <Container>
      <SmallerTitle>Subject</SmallerTitle>
      <ConfigRow>
        <ConfigLabel>Subject ID:</ConfigLabel>
        <CommittableNumericInput
          value={metadata.subject_id}
          onCommit={handleSubjectIdCommit}
          prefix="S"
          maxLength={3}
          placeholder="001"
          disabled={isSessionRunning}
        />
      </ConfigRow>
      <ConfigRow>
        <ConfigLabel>Notes:</ConfigLabel>
        <CommittableTextInput
          value={metadata.notes}
          onCommit={handleNotesCommit}
          placeholder="Enter notes"
          disabled={isSessionRunning}
          multiline={true}
          width="315px"
        />
      </ConfigRow>
    </Container>
  )
}