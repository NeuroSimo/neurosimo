import React, { useState, useEffect, ReactNode } from 'react'
import { Topic } from 'roslib'

import { ros } from 'ros/ros'


export interface ExperimentState extends ROSLIB.Message {
  stage_name: string
  stage_index: number
  total_stages: number
  trial: number
  total_trials_in_stage: number
  ongoing: boolean
  in_rest: boolean
  paused: boolean
  experiment_time: number
  stage_start_time: number
  stage_elapsed_time: number
  rest_duration: number
  rest_elapsed: number
  rest_remaining: number
  next_stage_name: string
  next_is_rest: boolean
}

interface ExperimentContextType {
  experimentState: ExperimentState | null
  setExperimentState: React.Dispatch<React.SetStateAction<ExperimentState | null>>
}

const defaultExperimentState: ExperimentContextType = {
  experimentState: null,
  setExperimentState: () => {
    console.warn('setExperimentState is not yet initialized.')
  },
}

export const ExperimentContext = React.createContext<ExperimentContextType>(defaultExperimentState)

interface ExperimentProviderProps {
  children: ReactNode
}

export const ExperimentProvider: React.FC<ExperimentProviderProps> = ({ children }) => {
  const [experimentState, setExperimentState] = useState<ExperimentState | null>(null)

  useEffect(() => {
    /* Subscriber for experiment state. */
    const experimentStateSubscriber = new Topic<ExperimentState>({
      ros: ros,
      name: '/pipeline/experiment_state',
      messageType: 'pipeline_interfaces/ExperimentState',
    })

    experimentStateSubscriber.subscribe((message) => {
      setExperimentState(message)
    })

    /* Unsubscribers */
    return () => {
      experimentStateSubscriber.unsubscribe()
    }
  }, [])

  return (
    <ExperimentContext.Provider
      value={{
        experimentState,
        setExperimentState,
      }}
    >
      {children}
    </ExperimentContext.Provider>
  )
}