import React, { useContext, useEffect, useState } from 'react'
import styled from 'styled-components'

import { StyledPanel, ConfigRow, ConfigLabel, Select } from 'styles/General'

import { SmallerTitle } from 'styles/ExperimentStyles'

import { ConfigContext, TargetingAlgorithms } from 'providers/ConfigProvider'

const InputRow = styled.div`
  display: flex;
  justify-content: flex-start;
  align-items: center;
  gap: 10px;
  margin-bottom: 10px;
`

const Label = styled.label`
  width: 150px;
  text-align: left;
  margin-right: 10px;
  margin-left: 30px;
  display: inline-block;
`

const Input = styled.input`
  width: 300px;
  padding: 8px;
  border: 1px solid #ccc;
  border-radius: 4px;
  outline: none;
  transition: background-color 0.2s;

  &:focus {
    background-color: transparent;
  }
`

/* Config panels */
const ConfigPanel = styled.div`
  display: grid;
  grid-template-rows: repeat(1, 1fr);
  grid-template-columns: repeat(2, 1fr);
  width: 600px;
  height: 550px;
  gap: 20px;
`

const TargetingPanel = styled(StyledPanel)`
  grid-row: 1 / 2;
  grid-column: 1 / 2;
  width: 280px;
  height: 110px;
`

export const ConfigView = () => {
  const { targetingAlgorithm, setTargetingAlgorithm } = useContext(ConfigContext)

  const changeTargetingAlgorithm = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const newTargetingAlgorithm = parseInt(event.target.value, 10)
    setTargetingAlgorithm(newTargetingAlgorithm)
  }

  return (
    <>
      <ConfigPanel>
        <TargetingPanel>
          <SmallerTitle>Targeting</SmallerTitle>
          <ConfigRow>
            <ConfigLabel>Algorithm:</ConfigLabel>
            <Select onChange={changeTargetingAlgorithm} value={targetingAlgorithm}>
              {Object.entries(TargetingAlgorithms).map(([key, value]) => (
                <option key={value} value={value}>
                  {key
                    .split('_')
                    .map((word) => word.charAt(0).toUpperCase() + word.slice(1).toLowerCase())
                    .join(' ')}
                </option>
              ))}
            </Select>
          </ConfigRow>
        </TargetingPanel>
      </ConfigPanel>
    </>
  )
}
