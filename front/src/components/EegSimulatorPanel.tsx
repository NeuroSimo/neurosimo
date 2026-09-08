import React, { useContext, useState, useEffect } from 'react'
import styled from 'styled-components'

import { ValidatedInput } from 'components/ValidatedInput'
import { FolderTerminalButtons } from 'components/FolderTerminalButtons'

import {
  StyledPanel,
  Select,
  ConfigRow,
  ConfigLabel,
  ConfigValue,
  CONFIG_PANEL_WIDTH,
  StyledButton,
} from 'styles/General'

import { EegSimulatorContext, DataSourceStateValue } from 'providers/EegSimulatorProvider'
import { EegStreamContext } from 'providers/EegStreamProvider'
import { useSessionConfig } from 'providers/SessionConfigProvider'
import { useSession, SessionStateValue } from 'providers/SessionProvider'
import { formatTime, formatFrequency } from 'utils/utils'
import { useDefaultToFirstOption } from 'utils/useDefaultToFirstOption'
import { HealthcheckContext } from 'providers/HealthProvider'
import { getDatasetInfoRos, DatasetInfo } from 'ros/eeg_simulator'

const SimulatorPanel = styled(StyledPanel)`
  width: ${CONFIG_PANEL_WIDTH - 30}px;
  position: static;
  display: flex;
  flex-direction: column;
  gap: 4px;
`

const DatasetSelect = styled(Select)`
  margin-left: 6px;
  width: 170px;
`

const CompactRow = styled(ConfigRow)`
  margin-bottom: 2px;
  gap: 4px;
`

/* How long a dataset info fetch may take before the wait is reported to the user.

   Switching datasets is normally answered well within this, and blanking the fields for that
   moment reads as a flicker, so the previously fetched information is left on display until
   the new one arrives. Datasets that genuinely take long to read are common enough that the
   stale figures cannot simply be left standing, hence the deadline. */
const DATASET_INFO_REPORT_WAIT_AFTER_MS = 200


export const EegSimulatorPanel: React.FC<{ isGrayedOut: boolean }> = ({ isGrayedOut }) => {
  const { eegSimulatorStatus } = useContext(HealthcheckContext)
  const {
    datasetList,
    dataset,
    startTime,
    playbackSpeed,
    dataSourceState,
  } = useContext(EegSimulatorContext)
  const { eegDeviceInfo } = useContext(EegStreamContext)
  const { setSimulatorDataset, setSimulatorStartTime, setSimulatorPlaybackSpeed, isDraftLoaded } = useSessionConfig()
  const { sessionState } = useSession()

  /* The dataset information on display. It is replaced when a fetch resolves and dropped when
     a fetch has been outstanding past the deadline above; selecting a dataset does not disturb
     it. Keeping it in one piece of state is what makes the display change exactly once per
     fetch: a second, separate flag for the wait would have renders where the two disagree.

     undefined means there is nothing to show — no fetch has resolved yet, or the wait for one
     has passed the deadline. null is a fetch that failed, which is distinct from that. */
  const [displayedDatasetInfo, setDisplayedDatasetInfo] = useState<DatasetInfo | null | undefined>(undefined)

  const isSessionRunning = sessionState.state === SessionStateValue.RUNNING
  const isEegStreaming = eegDeviceInfo?.is_streaming || false

  const selectDataset = (filename: string) =>
    setSimulatorDataset(filename, () => {
      console.log('Dataset set to ' + filename)
    })

  const hasDataset = useDefaultToFirstOption(
    dataset,
    datasetList,
    selectDataset,
    isDraftLoaded && !isSessionRunning && !isEegStreaming
  )

  // Fetch dataset info when dataset changes
  useEffect(() => {
    if (!dataset || dataset.trim() === '') {
      setDisplayedDatasetInfo(null)
      return
    }

    let cancelled = false

    /* Until the deadline expires, the information fetched for the previous dataset stays up. */
    const deadline = window.setTimeout(() => {
      if (!cancelled) {
        setDisplayedDatasetInfo(undefined)
      }
    }, DATASET_INFO_REPORT_WAIT_AFTER_MS)

    getDatasetInfoRos(dataset, (datasetInfo) => {
      if (cancelled) return
      if (!datasetInfo) {
        console.error('Failed to get dataset info for:', dataset)
      }
      window.clearTimeout(deadline)
      setDisplayedDatasetInfo(datasetInfo)
    })

    return () => {
      cancelled = true
      window.clearTimeout(deadline)
    }
  }, [dataset])

  // Handle arrow key navigation for dataset selection
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      // Only handle arrow keys when not typing in inputs and not disabled
      if (isSessionRunning || isEegStreaming) return

      // Skip if user is typing in an input field
      const target = event.target as HTMLElement
      if (target.tagName === 'INPUT' || target.tagName === 'TEXTAREA') return

      const currentIndex = datasetList.indexOf(dataset)
      if (currentIndex === -1) return

      if (event.key === 'ArrowUp' && currentIndex > 0) {
        event.preventDefault()
        // Blur any currently focused element to prevent focus outline
        if (document.activeElement instanceof HTMLElement) {
          document.activeElement.blur()
        }
        const newIndex = currentIndex - 1
        setSimulatorDataset(datasetList[newIndex], () => {
          console.log('Dataset changed to ' + datasetList[newIndex] + ' via arrow key')
        })
      } else if (event.key === 'ArrowDown' && currentIndex < datasetList.length - 1) {
        event.preventDefault()
        // Blur any currently focused element to prevent focus outline
        if (document.activeElement instanceof HTMLElement) {
          document.activeElement.blur()
        }
        const newIndex = currentIndex + 1
        setSimulatorDataset(datasetList[newIndex], () => {
          console.log('Dataset changed to ' + datasetList[newIndex] + ' via arrow key')
        })
      }
    }

    document.addEventListener('keydown', handleKeyDown)
    return () => document.removeEventListener('keydown', handleKeyDown)
  }, [dataset, datasetList, isSessionRunning, isEegStreaming, setSimulatorDataset])

  const setDataset = (event: React.ChangeEvent<HTMLSelectElement>) => {
    selectDataset(event.target.value)
  }

  const isLoadingDatasetInfo = displayedDatasetInfo === undefined
  const selectedDatasetInfo = displayedDatasetInfo ?? null

  const setStartTime = (startTime: number) => {
    if (startTime < 0 || startTime > (selectedDatasetInfo?.duration || 0)) {
      console.error('Start time must be between 0 and ' + selectedDatasetInfo?.duration + ' seconds')
      return
    }
    setSimulatorStartTime(startTime, () => {
      console.log('Start time set to ' + startTime)
    })
  }

  const setPlaybackSpeed = (speed: number) => {
    if (speed <= 0) {
      console.error('Playback speed must be greater than 0')
      return
    }
    setSimulatorPlaybackSpeed(speed, () => {
      console.log('Playback speed set to ' + speed)
    })
  }

  const dataSourceStateLabel =
    dataSourceState === DataSourceStateValue.RUNNING
      ? 'Running'
      : dataSourceState === DataSourceStateValue.LOADING
      ? 'Loading'
      : dataSourceState === DataSourceStateValue.ERROR
      ? 'Error'
      : 'Ready'

  /* A value that is not among the options makes the browser display the first option while
     the configuration still holds the original value. Show the mismatch instead. */
  return (
    <SimulatorPanel isGrayedOut={isGrayedOut}>
      <ConfigRow style={{ justifyContent: 'space-between' }}>
        <ConfigLabel>Dataset</ConfigLabel>
        <DatasetSelect onChange={setDataset} value={hasDataset ? dataset : ''} disabled={isSessionRunning || isEegStreaming}>
          {!hasDataset && (
            <option value="" disabled>
              —
            </option>
          )}
          {datasetList.map((datasetFilename: typeof datasetList[number], index: number) => (
            <option key={index} value={datasetFilename}>
              {datasetFilename}
            </option>
          ))}
        </DatasetSelect>
      </ConfigRow>
      <CompactRow>
        <ConfigLabel>Duration</ConfigLabel>
        <ConfigValue>
          {isLoadingDatasetInfo
            ? '...'
            : selectedDatasetInfo?.loop
            ? 'Continuous'
            : `${formatTime(selectedDatasetInfo?.duration)}${selectedDatasetInfo?.trial_count ? `, ${selectedDatasetInfo.trial_count} pulses` : ''}`}
        </ConfigValue>
      </CompactRow>

      <div style={{ height: '8px' }} />

      <CompactRow>
        <ConfigLabel>Sampling rate</ConfigLabel>
        <ConfigValue>{isLoadingDatasetInfo ? '' : formatFrequency(selectedDatasetInfo?.sampling_frequency)}</ConfigValue>
      </CompactRow>
      <CompactRow>
        <ConfigLabel>Channels</ConfigLabel>
      </CompactRow>
      <CompactRow>
        <ConfigLabel style={{ paddingLeft: 10 }}>EEG</ConfigLabel>
        <ConfigValue>{isLoadingDatasetInfo ? '' : selectedDatasetInfo?.num_eeg_channels}</ConfigValue>
      </CompactRow>
      <CompactRow>
        <ConfigLabel style={{ paddingLeft: 10 }}>EMG</ConfigLabel>
        <ConfigValue>{isLoadingDatasetInfo ? '' : selectedDatasetInfo?.num_emg_channels}</ConfigValue>
      </CompactRow>

      <div style={{ height: '8px' }} />

      <CompactRow style={{ justifyContent: 'space-between' }}>
        <ConfigLabel>Start time (s)</ConfigLabel>
        <div style={{ marginRight: 20 }}>
          <ValidatedInput
            type='number'
            value={startTime}
            formatValue={(value) => value.toFixed(1)}
            min={0}
            max={selectedDatasetInfo?.duration || 0}
            onChange={setStartTime}
            disabled={isSessionRunning || isEegStreaming}
            width="60px"
          />
        </div>
      </CompactRow>
      <CompactRow style={{ justifyContent: 'space-between' }}>
        <ConfigLabel>Playback speed</ConfigLabel>
        <div style={{ marginRight: 20 }}>
          <ValidatedInput
            type='number'
            value={playbackSpeed}
            formatValue={(value) => value.toFixed(1)}
            min={0.01}
            step={0.1}
            onChange={setPlaybackSpeed}
            disabled={isSessionRunning || isEegStreaming}
            width="60px"
          />
        </div>
      </CompactRow>
      <CompactRow>
        <ConfigLabel>Status</ConfigLabel>
        <ConfigValue>{dataSourceStateLabel}</ConfigValue>
      </CompactRow>
      <CompactRow style={{ justifyContent: 'flex-end', paddingRight: '10px', gap: '6px' }}>
        <FolderTerminalButtons folderName="eeg_simulator" />
      </CompactRow>
    </SimulatorPanel>
  )
}