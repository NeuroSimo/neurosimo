import React, { useEffect, useState } from 'react'
import styled from 'styled-components'

import { TabBar } from 'styles/General'

import {
  chargeFeedbackSubscriber,
  dischargeFeedbackSubscriber,
  pulseFeedbackSubscriber,
  triggerOutFeedbackSubscriber,
} from 'ros/subscribers/feedback'

import {
  PulseFeedbackMessage,
  Feedback,
  ChargeFeedbackMessage,
  DischargeFeedbackMessage,
  TriggerOutFeedbackMessage,
} from 'types/event'

import { SystemState } from 'components/SystemState'
import { DeviceControl } from 'components/DeviceControl'
import { EventFeedbacks } from 'components/EventFeedbacks'

const initialState = {
  channel_states: [],
  system_error_cumulative: {
    heartbeat_error: false,
    latched_fault_error: false,
    powersupply_error: false,
    safety_bus_error: false,
    coil_error: false,
    emergency_button_error: false,
    door_error: false,
    charger_overvoltage_error: false,
    charger_overtemperature_error: false,
    monitored_voltage_over_maximum_error: false,
    patient_safety_error: false,
    device_safety_error: false,
    charger_powerup_error: false,
    opto_error: false,
    charger_power_enabled_twice_error: false,
  },
  system_error_current: {
    heartbeat_error: false,
    latched_fault_error: false,
    powersupply_error: false,
    safety_bus_error: false,
    coil_error: false,
    emergency_button_error: false,
    door_error: false,
    charger_overvoltage_error: false,
    charger_overtemperature_error: false,
    monitored_voltage_over_maximum_error: false,
    patient_safety_error: false,
    device_safety_error: false,
    charger_powerup_error: false,
    opto_error: false,
    charger_power_enabled_twice_error: false,
  },
  system_error_emergency: {
    heartbeat_error: false,
    latched_fault_error: false,
    powersupply_error: false,
    safety_bus_error: false,
    coil_error: false,
    emergency_button_error: false,
    door_error: false,
    charger_overvoltage_error: false,
    charger_overtemperature_error: false,
    monitored_voltage_over_maximum_error: false,
    patient_safety_error: false,
    device_safety_error: false,
    charger_powerup_error: false,
    opto_error: false,
    charger_power_enabled_twice_error: false,
  },
  startup_error: {
    value: 0,
  },
  device_state: {
    value: 0,
  },
  session_state: {
    value: 0,
  },
  time: 0,
}

export const SystemView = () => {
  const [activeTab, setActiveTab] = useState<'overview' | 'diagnostics'>('overview')

  const [feedback, setFeedback] = useState<Feedback>()

  useEffect(() => {
    console.log('Subscribed')
    pulseFeedbackSubscriber.subscribe(pulseFeedbackCallback)
    chargeFeedbackSubscriber.subscribe(chargeFeedbackCallback)
    dischargeFeedbackSubscriber.subscribe(dischargeFeedbackCallback)
    triggerOutFeedbackSubscriber.subscribe(triggerOutFeedbackCallback)
  }, [])

  const pulseFeedbackCallback = (message: PulseFeedbackMessage) => {
    setFeedback({
      ...message,
      type: 'pulse',
    })
  }

  const chargeFeedbackCallback = (message: ChargeFeedbackMessage) => {
    setFeedback({
      ...message,
      type: 'charge',
    })
  }

  const dischargeFeedbackCallback = (message: DischargeFeedbackMessage) => {
    setFeedback({
      ...message,
      type: 'discharge',
    })
  }

  const triggerOutFeedbackCallback = (message: TriggerOutFeedbackMessage) => {
    setFeedback({
      ...message,
      type: 'triggerOut',
    })
  }

  return (
    <>
      <TabBar>
        <a href='#' onClick={() => setActiveTab('overview')} className={activeTab === 'overview' ? 'active' : ''}>
          Overview
        </a>
        <a href='#' onClick={() => setActiveTab('diagnostics')} className={activeTab === 'diagnostics' ? 'active' : ''}>
          Diagnostics
        </a>
      </TabBar>
      <Wrapper>
        {activeTab === 'overview' && (
          <>
            <PanelA>
              <DeviceControl />
            </PanelA>
            <VerticalDividedPanelB>
              <SubPanel>
                <SystemState />
              </SubPanel>
            </VerticalDividedPanelB>
            <PanelC>
              <EventFeedbacks feedback={feedback} />
            </PanelC>
          </>
        )}
      </Wrapper>
    </>
  )
}

const Wrapper = styled.div`
  display: grid;
  grid-template-columns: 1fr 1fr;
  grid-template-rows: auto 1fr;
  gap: 1rem;
  padding: 1rem;
`

const styledPanel = `
  padding: 1rem;
  border-radius: 5px;
  background-color: #f7f7f7;
  box-shadow: 0px 3px 6px rgba(0, 0, 0, 0.1);/
`

const PanelA = styled.div`
  grid-row: 1 / 2;
  grid-column: 1 / 2;
  ${styledPanel}
`

const VerticalDividedPanelB = styled.div`
  grid-row: 1 / 2;
  grid-column: 2 / 3;
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
  height: 96%;
  ${styledPanel}
`

const SubPanel = styled(PanelA)`
  margin: 0;
`

const PanelC = styled.div`
  grid-row: 2 / 3;
  grid-column: 1 / 3;
  ${styledPanel}
`
