import React, { useEffect, useRef, useState } from 'react'
import styled from 'styled-components'

import { EegBatchMessage, MTMSEvent, MTMSEventMessage } from 'types/eeg'
import { Datapoint } from 'components/EegChartStreaming'
import { WebGLPlot } from 'components/WebGLPlot'

import { eegDataSubscriber } from 'ros/ros'
import { eventSubscriber } from 'ros/subscribers/feedback'

import { objectKeysToCamelCase } from 'utils'

const DataVisualizeWebGL = () => {
  const minYRef = useRef(3550)
  const maxYRef = useRef(3620)

  const targetYMin = -1
  const targetYMax = 1

  const [latestBatch, setLatestBatch] = useState<Datapoint[]>([])

  const [latestEvent, setLatestEvent] = useState<MTMSEvent>()

  const [events, setEvents] = useState<MTMSEvent[]>([])
  const [pulsePoints, setPulsePoints] = useState<Datapoint[]>([])
  const [chargePoints, setChargePoints] = useState<Datapoint[]>([])
  const [dischargePoints, setDischargePoints] = useState<Datapoint[]>([])
  const [triggerOutPoints, setTriggerOutPoints] = useState<Datapoint[]>([])

  const [latestTimestamps, setLatestTimestamps] = useState<number[]>([])

  useEffect(() => {
    console.log('Subscribing to eeg data')
    eegDataSubscriber.subscribe(newEegBatch)
    eventSubscriber.subscribe(newEvent)
  }, [])

  const c3 = (datapoint: number[]) =>
    datapoint[4] - 0.25 * (datapoint[21] + datapoint[23] + datapoint[25] + datapoint[27])

  const scaleY = (point: number) =>
    ((point - minYRef.current) / (maxYRef.current - minYRef.current)) * (targetYMax - targetYMin) + targetYMin

  const newEegBatch = (message: EegBatchMessage) => {
    const data = []
    const eegTimestamps: number[] = []
    for (let i = 0; i < message.batch.length; i++) {
      const point = message.batch[i]
      eegTimestamps.push(point.time)

      const filtered = c3(point.eeg_data)
      const y = scaleY(filtered)

      const finalPoint = {
        x: 0,
        y,
      }
      data.push(finalPoint)
    }

    setLatestTimestamps(eegTimestamps)
    setLatestBatch(data)
  }

  useEffect(() => {
    const initData = () => {
      return latestTimestamps.map((p) => {
        return {
          x: 0,
          y: -1,
        }
      })
    }

    const newPulseData = initData()
    const newChargeData = initData()
    const newDischargeData = initData()
    const newTriggerOutData = initData()

    const allData = [newPulseData, newChargeData, newDischargeData, newTriggerOutData]

    const newEvents: MTMSEvent[] = []

    for (let j = 0; j < events.length; j++) {
      const event = events[j]
      let removeThisEvent = false
      for (let i = 0; i < latestTimestamps.length; i++) {
        const ts = latestTimestamps[i]
        if (event.whenToExecute < ts) {
          console.log(`event: ${event.eventType}, ${event.whenToExecute}, ${ts}`)
          allData[event.eventType][i].y = 1
          removeThisEvent = true
          break
        }
      }

      if (!removeThisEvent) {
        newEvents.push(event)
      }
    }

    setEvents(() => newEvents)

    setChargePoints(newChargeData)
    setPulsePoints(newPulseData)
    setDischargePoints(newDischargeData)
    setTriggerOutPoints(newTriggerOutData)
  }, [latestTimestamps])

  useEffect(() => {
    if (latestEvent) {
      const newEvents = [...events, latestEvent]
      setEvents(() => newEvents)
    }
  }, [latestEvent])

  const newEvent = (message: MTMSEventMessage) => {
    const camelCased: MTMSEvent = objectKeysToCamelCase(message)

    setLatestEvent(camelCased)
  }

  return (
    <ChartContainer>
      <label htmlFor='chart-max-input'>Y max: </label>
      <AxisLimit
        defaultValue={maxYRef.current}
        type='number'
        id='chart-max-input'
        onChange={(event) => (maxYRef.current = Number(event.target.value))}
      />
      <br />
      <label htmlFor='chart-min-input'>Y min: </label>
      <AxisLimit
        defaultValue={minYRef.current}
        type='number'
        id='chart-min-input'
        onChange={(event) => (minYRef.current = Number(event.target.value))}
      />
      <WebGLPlot
        eegData={latestBatch}
        pulseData={pulsePoints}
        chargeData={chargePoints}
        dischargeData={dischargePoints}
        triggerOutData={triggerOutPoints}
      />
    </ChartContainer>
  )
}

const ChartContainer = styled.div``

const AxisLimit = styled.input`
  max-width: 50px;
`

export default DataVisualizeWebGL
