import React, { useContext, useEffect, useState } from 'react'
import styled from 'styled-components'
import { ExperimentContext } from 'providers/ExperimentProvider'

const DragBar = styled.div`
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  height: 20px;
  background-color: transparent;
  -webkit-app-region: drag;
  z-index: 9999;
`

const FullScreenContainer = styled.div`
  width: 100vw;
  height: 100vh;
  display: flex;
  align-items: center;
  justify-content: center;
  background-color: black;
  color: white;
  font-family: Arial, sans-serif;
  font-size: 48px;
  font-weight: bold;
`

const Cross = styled.div`
  font-size: 120px;
  color: white;
`

const RestText = styled.div`
  font-size: 72px;
  text-align: center;
`

const Timer = styled.div`
  font-size: 96px;
  margin-top: 20px;
`

export const DetachedExperimentView: React.FC = () => {
  const { experimentState } = useContext(ExperimentContext)
  
  const formatTime = (seconds: number) => {
    const mins = Math.floor(seconds / 60)
    const secs = Math.floor(seconds % 60)
    return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`
  }

  if (!experimentState || !experimentState.ongoing) {
    return (
      <>
        <DragBar />
        <FullScreenContainer />
      </>
    )
  }

  if (experimentState.paused) {
    return (
      <>
        <DragBar />
        <FullScreenContainer>
          Paused
        </FullScreenContainer>
      </>
    )
  }

  if (experimentState.in_rest) {
    return (
      <>
        <DragBar />
        <FullScreenContainer>
          <div>
            <RestText>Rest</RestText>
            <RestText style={{ fontSize: '48px', marginTop: '10px' }}>
              Resuming in
            </RestText>
            <Timer>{formatTime(experimentState.rest_remaining)}</Timer>
          </div>
        </FullScreenContainer>
      </>
    )
  }

  return (
    <>
      <DragBar />
      <FullScreenContainer>
        <Cross>+</Cross>
      </FullScreenContainer>
    </>
  )
}
