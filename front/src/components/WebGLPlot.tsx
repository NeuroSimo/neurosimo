import React, { useEffect, useRef, useState } from 'react'
import { WebglPlot, WebglLine, ColorRGBA } from 'webgl-plot'
import { Datapoint, EegChartProps } from './EegChartStreaming'
import styled from 'styled-components'

type WebGLPlotProps = {
  eegData: Datapoint[]
  pulseData: Datapoint[]
  chargeData: Datapoint[]
  dischargeData: Datapoint[]
  triggerOutData: Datapoint[]
}

export const WebGLPlot = ({ eegData, pulseData, chargeData, dischargeData, triggerOutData }: WebGLPlotProps) => {
  const canvasMain = useRef<HTMLCanvasElement>(null)

  const [webGLPlot, setWebGLPlot] = useState<WebglPlot>()
  const [eegLine, setEegLine] = useState<WebglLine>()
  const [pulseLine, setPulseLine] = useState<WebglLine>()
  const [triggerOutLine, setTriggerOutLine] = useState<WebglLine>()
  const [chargeLine, setChargeLine] = useState<WebglLine>()
  const [dischargeLine, setDischargeLine] = useState<WebglLine>()

  useEffect(() => {
    if (canvasMain.current) {
      const devicePixelRatio = window.devicePixelRatio || 1
      canvasMain.current.width = canvasMain.current.clientWidth * devicePixelRatio
      canvasMain.current.height = canvasMain.current.clientHeight * devicePixelRatio

      const webglp = new WebglPlot(canvasMain.current)
      setWebGLPlot(webglp)
      const numX = 2500

      const line1 = new WebglLine(new ColorRGBA(1, 0, 0, 1), numX)
      const line2 = new WebglLine(new ColorRGBA(0, 0, 1, 1), numX)
      const line3 = new WebglLine(new ColorRGBA(0.9, 0.8, 0.2, 1), numX)
      const line4 = new WebglLine(new ColorRGBA(0, 1, 0, 1), numX)
      const line5 = new WebglLine(new ColorRGBA(0.7, 0.45, 0.3, 1), numX)

      webglp.addLine(line1)
      webglp.addLine(line2)
      webglp.addLine(line3)
      webglp.addLine(line4)
      webglp.addLine(line5)

      line1.arrangeX()
      line2.arrangeX()
      line3.arrangeX()
      line4.arrangeX()
      line5.arrangeX()

      setWebGLPlot(webglp)
      setEegLine(line1)
      setPulseLine(line2)
      setChargeLine(line3)
      setTriggerOutLine(line4)
      setDischargeLine(line5)
    }
  }, [])

  useEffect(() => {
    let id = 0
    let renderPlot = () => {
      const y = new Float32Array(eegData.map((datapoint) => datapoint.y))
      eegLine?.shiftAdd(y)
      webGLPlot?.update()
    }

    id = requestAnimationFrame(renderPlot)

    return () => {
      // eslint-disable-next-line @typescript-eslint/no-empty-function
      renderPlot = () => {}
      cancelAnimationFrame(id)
    }
  }, [eegData])

  useEffect(() => {
    let id = 0
    let renderPlot = () => {
      const y = new Float32Array(pulseData.map((datapoint) => datapoint.y))
      pulseLine?.shiftAdd(y)
      webGLPlot?.update()
    }

    id = requestAnimationFrame(renderPlot)

    return () => {
      // eslint-disable-next-line @typescript-eslint/no-empty-function
      renderPlot = () => {}
      cancelAnimationFrame(id)
    }
  }, [pulseData])

  useEffect(() => {
    let id = 0
    let renderPlot = () => {
      const y = new Float32Array(chargeData.map((datapoint) => datapoint.y))
      chargeLine?.shiftAdd(y)
      webGLPlot?.update()
    }

    id = requestAnimationFrame(renderPlot)

    return () => {
      // eslint-disable-next-line @typescript-eslint/no-empty-function
      renderPlot = () => {}
      cancelAnimationFrame(id)
    }
  }, [chargeData])

  useEffect(() => {
    let id = 0
    let renderPlot = () => {
      const y = new Float32Array(dischargeData.map((datapoint) => datapoint.y))
      dischargeLine?.shiftAdd(y)
      webGLPlot?.update()
    }

    id = requestAnimationFrame(renderPlot)

    return () => {
      // eslint-disable-next-line @typescript-eslint/no-empty-function
      renderPlot = () => {}
      cancelAnimationFrame(id)
    }
  }, [dischargeData])

  useEffect(() => {
    let id = 0
    let renderPlot = () => {
      const y = new Float32Array(triggerOutData.map((datapoint) => datapoint.y))
      triggerOutLine?.shiftAdd(y)
      webGLPlot?.update()
    }

    id = requestAnimationFrame(renderPlot)

    return () => {
      // eslint-disable-next-line @typescript-eslint/no-empty-function
      renderPlot = () => {}
      cancelAnimationFrame(id)
    }
  }, [triggerOutData])

  return (
    <div>
      <StyledCanvas ref={canvasMain} />
    </div>
  )
}

const StyledCanvas = styled.canvas`
  width: 100%;
  height: 70vh;
  border: 1px solid black;
`
