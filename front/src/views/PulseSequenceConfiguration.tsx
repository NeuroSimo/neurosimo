import React from 'react'
import { ChannelInfoWithEnabled } from '../types/pulseSequence'
import styled from 'styled-components'
import { useAppDispatch, useAppSelector } from '../providers/reduxHooks'
import {
  setChannels,
  setIbi,
  setIsis,
  setIti,
  setNofBurstsInTrains,
  setNofPulsesInBursts,
  setNofTrains,
} from 'reducers/sessionReducer'

const Channel = (props: ChannelInfoWithEnabled) => {
  const { channelIndex, enabled, voltage } = props
  const { channels } = useAppSelector((state) => state.session)
  const dispatch = useAppDispatch()

  const handleOnChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    const target = event.target.name
    const value =
      event.target.type === 'checkbox'
        ? event.target.checked
        : event.target.type === 'number'
        ? Number(event.target.value)
        : event.target.value

    const newChannels = [...channels]
    newChannels[channelIndex - 1] = {
      ...props,
      [target]: value,
    }
    dispatch(setChannels(newChannels))
  }

  return (
    <ChannelContainer enabled={enabled}>
      <input name='enabled' type='checkbox' checked={enabled} onChange={handleOnChange} />
      Channel {channelIndex} <label htmlFor='voltage'>Voltage: </label>
      <input name='voltage' type='number' disabled={!enabled} value={voltage} onChange={handleOnChange} />
    </ChannelContainer>
  )
}

const ChannelContainer = styled.div<{
  enabled: boolean
}>`
  color: ${(p) => (p.enabled ? p.theme.colors.primary : p.theme.colors.grey)};
`
/** TODO: button to add additional channels */
const PulseSequenceConfiguration = () => {
  const { channels, iti, ibi, nofBurstsInTrains, nofTrains, isis, nofPulsesInBursts } = useAppSelector(
    (state) => state.session,
  )
  const dispatch = useAppDispatch()

  const isiChangeHandler = (value: string, index: number) => {
    const newIsis = [...isis]
    newIsis[index] = Number(value)
    dispatch(setIsis(newIsis))
  }

  const nofPulsesInBurstChangeHandler = (value: number) => {
    const last = isis.length > 0 ? isis[isis.length - 1] : 500

    const nofElementsToAdd = value - isis.length - 1

    let newIsis: number[]
    if (nofElementsToAdd > 0) {
      const elementsToAdd = Array(nofElementsToAdd).fill(last)
      newIsis = isis.concat(...elementsToAdd)
    } else {
      newIsis = isis.slice(0, value - 1)
    }

    dispatch(setNofPulsesInBursts(value))
    dispatch(setIsis(newIsis))
  }

  return (
    <div>
      <h2>Pulse sequence configuration</h2>
      <label htmlFor='iti'>
        Inter-train interval (iti) (<span>&#181;</span>){' '}
      </label>

      <input
        name='iti'
        type='number'
        value={iti}
        min={0}
        onChange={(event) => dispatch(setIti(Number(event.target.value)))}
      />
      <br />
      <label htmlFor='ibi'>
        Inter-burst interval (ibi) (<span>&#181;</span>){' '}
      </label>
      <input
        name='ibi'
        type='number'
        value={ibi}
        min={0}
        onChange={(event) => dispatch(setIbi(Number(event.target.value)))}
      />
      <br />

      <label htmlFor='nofPulsesInBursts'>Number of pulses in bursts </label>
      <input
        name='nofPulsesInBursts'
        type='number'
        min={0}
        value={nofPulsesInBursts}
        onChange={(event) => nofPulsesInBurstChangeHandler(Number(event.target.value))}
      />
      <br />
      <label htmlFor='nofBurstsInTrains'>Number of bursts in trains </label>
      <input
        name='nofBurstsInTrains'
        type='number'
        min={0}
        value={nofBurstsInTrains}
        onChange={(event) => dispatch(setNofBurstsInTrains(Number(event.target.value)))}
      />
      <br />
      <br />

      {isis.map((isi, index) => {
        return (
          <div key={`isi-${index}`}>
            <label htmlFor={`isi-${index}`}>
              Interstimulus interval (isi) {index + 1} (<span>&#181;</span>)
            </label>
            <input
              name={`isi-${index}`}
              type='number'
              min={0}
              value={isi}
              onChange={(event) => isiChangeHandler(event.target.value, index)}
            />
            <br />
          </div>
        )
      })}

      <br />

      <label htmlFor='nofTrains'>Number of trains </label>
      <input
        name='nofTrains'
        type='number'
        min={0}
        value={nofTrains}
        onChange={(event) => dispatch(setNofTrains(Number(event.target.value)))}
      />
      <br />
      <br />

      <h3>Channels</h3>
      <ChannelsContainer>
        {channels.map((channel) => (
          <Channel key={`channel-${channel.channelIndex}`} {...channel} />
        ))}
      </ChannelsContainer>

      <br />
    </div>
  )
}

const ChannelsContainer = styled.div``

export default PulseSequenceConfiguration
