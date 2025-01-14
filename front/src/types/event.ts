import ROSLIB from 'roslib'
import { ChargeError, DischargeError, PulseError, TriggerOutError } from './eventErrors'

interface Error {
  value: number
}

export type Feedback =
  | PulseFeedbackMessage
  | ChargeFeedbackMessage
  | DischargeFeedbackMessage
  | TriggerOutFeedbackMessage

export interface PulseFeedbackMessage extends ROSLIB.Message {
  id: number
  error: Error
  type: 'pulse'
}

export interface ChargeFeedbackMessage extends ROSLIB.Message {
  id: number
  error: Error
  type: 'charge'
}

export interface DischargeFeedbackMessage extends ROSLIB.Message {
  id: number
  error: Error
  type: 'discharge'
}

export interface TriggerOutFeedbackMessage extends ROSLIB.Message {
  id: number
  error: Error
  type: 'triggerOut'
}
