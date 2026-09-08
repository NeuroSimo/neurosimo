import React, { useState, useEffect, ReactNode, createContext, useContext } from 'react'

import { SessionConfigMessage } from 'ros/session'
import { useProjectFiles } from './ProjectFilesProvider'

// Structured parameter interfaces
interface MetadataParameters {
  subject_id: number
}

interface ModuleSelection {
  module: string
  enabled: boolean
}

interface PipelineParameters {
  decider: ModuleSelection
  preprocessor: ModuleSelection
  presenter: ModuleSelection
  experiment: {
    protocol: string
  }
}

export type RuntimeParameterValue = number | string | boolean

export type RuntimeParameters = Record<string, RuntimeParameterValue>

interface SimulatorParameters {
  dataset_filename: string
  start_time: number
  playback_speed: number
}

interface ReplayParameters {
  bag_id: string
  play_preprocessed: boolean
}

/* The session draft: what the user has configured but has not yet started a session with.
   It is UI state, owned by the UI and persisted locally per project. The backend learns
   about it only when a session is started, as the payload of the start request. */
interface SessionDraft {
  metadata: MetadataParameters
  pipeline: PipelineParameters
  simulator: SimulatorParameters
  replay: ReplayParameters

  /* Notes, kept per subject so that switching between subjects retains what was written
     for each. */
  notesBySubject: Record<string, string>

  /* Runtime parameter values, kept per subject and protocol so that switching between
     subjects or protocols retains what was entered for each. Entries for combinations that
     are not currently selected are harmless: only the selected one is ever read. */
  runtimeParametersBySubjectAndProtocol: Record<string, Record<string, RuntimeParameters>>
}

const defaultDraft: SessionDraft = {
  metadata: {
    subject_id: 1,
  },
  pipeline: {
    decider: { module: 'example.py', enabled: true },
    preprocessor: { module: 'example.py', enabled: false },
    presenter: { module: 'example.py', enabled: false },
    experiment: { protocol: 'example.yaml' },
  },
  simulator: {
    dataset_filename: '',
    start_time: 0,
    playback_speed: 1,
  },
  replay: {
    bag_id: '',
    play_preprocessed: false,
  },
  notesBySubject: {},
  runtimeParametersBySubjectAndProtocol: {},
}

/* Subject ids are numbers, but object keys are strings. */
const subjectKey = (subjectId: number) => String(subjectId)

/* The notes written for the subject the draft currently names. */
const notesFor = (draft: SessionDraft): string => draft.notesBySubject[subjectKey(draft.metadata.subject_id)] ?? ''

/* The values entered for a given protocol, for the subject the draft currently names. */
const runtimeParametersFor = (draft: SessionDraft, protocol: string): RuntimeParameters =>
  draft.runtimeParametersBySubjectAndProtocol[subjectKey(draft.metadata.subject_id)]?.[protocol] ?? {}

const draftStorageKey = (project: string) => `neurosimo.sessionDraft.${project}`

const loadDraft = (project: string): SessionDraft => {
  try {
    const stored = localStorage.getItem(draftStorageKey(project))
    if (stored === null) {
      return defaultDraft
    }
    const parsed = JSON.parse(stored)
    /* Notes used to be a single value in the metadata; keep them by attributing them to the
       subject the draft was written for. */
    const { notes: legacyNotes, ...parsedMetadata } = parsed.metadata ?? {}
    const metadata = { ...defaultDraft.metadata, ...parsedMetadata }
    const legacyNotesBySubject =
      typeof legacyNotes === 'string' && legacyNotes !== '' ? { [subjectKey(metadata.subject_id)]: legacyNotes } : {}
    /* Merge over the defaults, so that a draft written before a field existed still loads. */
    return {
      ...defaultDraft,
      ...parsed,
      metadata,
      pipeline: { ...defaultDraft.pipeline, ...parsed.pipeline },
      simulator: { ...defaultDraft.simulator, ...parsed.simulator },
      replay: { ...defaultDraft.replay, ...parsed.replay },
      notesBySubject: parsed.notesBySubject ?? legacyNotesBySubject,
      runtimeParametersBySubjectAndProtocol: parsed.runtimeParametersBySubjectAndProtocol ?? {},
    }
  } catch (error) {
    console.warn(`Failed to load session draft for project '${project}':`, error)
    return defaultDraft
  }
}

const saveDraft = (project: string, draft: SessionDraft) => {
  try {
    localStorage.setItem(draftStorageKey(project), JSON.stringify(draft))
  } catch (error) {
    console.warn(`Failed to save session draft for project '${project}':`, error)
  }
}

interface SessionConfigContextType {
  // Structured parameter access
  metadata: MetadataParameters
  pipeline: PipelineParameters
  simulator: SimulatorParameters
  replay: ReplayParameters
  dataSource: string

  /* The notes written for the currently selected subject. */
  notes: string

  /* Runtime parameter values are addressed by protocol rather than only for the selected one,
     so that a caller showing a protocol's inputs can read and write that protocol's values even
     while the selection has already moved on. */
  getRuntimeParameters: (protocol: string) => RuntimeParameters

  /* Whether the draft for the active project has been loaded. Until it has, the setters
     below are no-ops, as there is no draft yet to write into. */
  isDraftLoaded: boolean

  // Convenience setters
  setSubjectId: (subjectId: number, callback?: () => void) => void
  setNotes: (notes: string, callback?: () => void) => void
  setDeciderModule: (module: string, callback?: () => void) => void
  setDeciderEnabled: (enabled: boolean, callback?: () => void) => void
  setPreprocessorModule: (module: string, callback?: () => void) => void
  setPreprocessorEnabled: (enabled: boolean, callback?: () => void) => void
  setPresenterModule: (module: string, callback?: () => void) => void
  setPresenterEnabled: (enabled: boolean, callback?: () => void) => void
  setExperimentProtocol: (protocol: string, callback?: () => void) => void
  setSimulatorDataset: (filename: string, callback?: () => void) => void
  setSimulatorStartTime: (startTime: number, callback?: () => void) => void
  setSimulatorPlaybackSpeed: (playbackSpeed: number, callback?: () => void) => void
  setBagId: (bagId: string, callback?: () => void) => void
  setPlayPreprocessed: (playPreprocessed: boolean, callback?: () => void) => void
  setDataSource: (dataSource: string, callback?: () => void) => void
  setRuntimeParameters: (protocol: string, params: RuntimeParameters, callback?: () => void) => void

  /* Build the SessionConfig message to send when starting a session. */
  buildSessionConfigMessage: () => SessionConfigMessage
}

// ESLint disable for intentionally empty functions used as defaults
/* eslint-disable @typescript-eslint/no-empty-function */
const noop = () => {}
/* eslint-enable @typescript-eslint/no-empty-function */

const emptySessionConfigMessage: SessionConfigMessage = {
  subject_id: 1,
  notes: '',
  decider_module: '',
  decider_enabled: false,
  preprocessor_module: '',
  preprocessor_enabled: false,
  presenter_module: '',
  presenter_enabled: false,
  protocol_filename: '',
  runtime_parameters: '{}',
  data_source: 'simulator',
  simulator_dataset_filename: '',
  simulator_start_time: 0,
  simulator_playback_speed: 1,
  replay_bag_id: '',
  replay_play_preprocessed: false,
}

const defaultSessionConfigState: SessionConfigContextType = {
  metadata: defaultDraft.metadata,
  pipeline: defaultDraft.pipeline,
  simulator: defaultDraft.simulator,
  replay: defaultDraft.replay,
  dataSource: 'simulator',
  notes: '',
  getRuntimeParameters: () => ({}),
  isDraftLoaded: false,
  setSubjectId: noop,
  setNotes: noop,
  setDeciderModule: noop,
  setDeciderEnabled: noop,
  setPreprocessorModule: noop,
  setPreprocessorEnabled: noop,
  setPresenterModule: noop,
  setPresenterEnabled: noop,
  setExperimentProtocol: noop,
  setSimulatorDataset: noop,
  setSimulatorStartTime: noop,
  setSimulatorPlaybackSpeed: noop,
  setBagId: noop,
  setPlayPreprocessed: noop,
  setDataSource: noop,
  setRuntimeParameters: noop,
  buildSessionConfigMessage: () => emptySessionConfigMessage,
}

export const SessionConfigContext = createContext<SessionConfigContextType>(defaultSessionConfigState)

interface SessionConfigProviderProps {
  children: ReactNode
}

export const SessionConfigProvider: React.FC<SessionConfigProviderProps> = ({ children }) => {
  /* The draft belongs to the project whose files are on display rather than to the active one:
     its selections name files, and are only meaningful next to the list they were made from. */
  const { project } = useProjectFiles()

  /* The draft is tagged with the project it belongs to, so that it is never persisted under
     a project it was not loaded for while a project switch is in progress. */
  const [draftState, setDraftState] = useState<{ project: string; draft: SessionDraft } | null>(null)

  /* Which data source the UI is showing. Deliberately not persisted: it is not a property
     of the project, and defaults to the simulator on every start. */
  const [dataSource, setDataSourceState] = useState<string>('simulator')

  /* Load the draft whenever the project changes. */
  useEffect(() => {
    if (project === '') {
      setDraftState(null)
      return
    }
    setDraftState({ project: project, draft: loadDraft(project) })
  }, [project])

  /* Persist the draft on every change. */
  useEffect(() => {
    if (draftState === null || draftState.project !== project) {
      return
    }
    saveDraft(draftState.project, draftState.draft)
  }, [draftState, project])

  const draft = draftState?.draft ?? defaultDraft
  const isDraftLoaded = draftState !== null && draftState.project === project

  const updateDraft = (update: (current: SessionDraft) => SessionDraft, callback?: () => void) => {
    setDraftState((current) => (current === null ? current : { ...current, draft: update(current.draft) }))
    if (callback) {
      callback()
    }
  }

  const protocol = draft.pipeline.experiment.protocol
  const notes = notesFor(draft)
  const runtimeParameters = runtimeParametersFor(draft, protocol)

  const getRuntimeParameters = (forProtocol: string) => runtimeParametersFor(draft, forProtocol)

  const setSubjectId = (subjectId: number, callback?: () => void) =>
    updateDraft((current) => ({ ...current, metadata: { ...current.metadata, subject_id: subjectId } }), callback)

  const setNotes = (nextNotes: string, callback?: () => void) =>
    updateDraft(
      (current) => ({
        ...current,
        notesBySubject: { ...current.notesBySubject, [subjectKey(current.metadata.subject_id)]: nextNotes },
      }),
      callback,
    )

  const setModuleSelection = (
    component: 'decider' | 'preprocessor' | 'presenter',
    selection: Partial<ModuleSelection>,
    callback?: () => void,
  ) =>
    updateDraft(
      (current) => ({
        ...current,
        pipeline: {
          ...current.pipeline,
          [component]: { ...current.pipeline[component], ...selection },
        },
      }),
      callback,
    )

  const setDeciderModule = (module: string, callback?: () => void) =>
    setModuleSelection('decider', { module }, callback)
  const setDeciderEnabled = (enabled: boolean, callback?: () => void) =>
    setModuleSelection('decider', { enabled }, callback)
  const setPreprocessorModule = (module: string, callback?: () => void) =>
    setModuleSelection('preprocessor', { module }, callback)
  const setPreprocessorEnabled = (enabled: boolean, callback?: () => void) =>
    setModuleSelection('preprocessor', { enabled }, callback)
  const setPresenterModule = (module: string, callback?: () => void) =>
    setModuleSelection('presenter', { module }, callback)
  const setPresenterEnabled = (enabled: boolean, callback?: () => void) =>
    setModuleSelection('presenter', { enabled }, callback)

  const setExperimentProtocol = (nextProtocol: string, callback?: () => void) =>
    updateDraft(
      (current) => ({
        ...current,
        pipeline: { ...current.pipeline, experiment: { protocol: nextProtocol } },
      }),
      callback,
    )

  const setSimulatorDataset = (filename: string, callback?: () => void) =>
    updateDraft(
      (current) => ({ ...current, simulator: { ...current.simulator, dataset_filename: filename } }),
      callback,
    )

  const setSimulatorStartTime = (startTime: number, callback?: () => void) =>
    updateDraft((current) => ({ ...current, simulator: { ...current.simulator, start_time: startTime } }), callback)

  const setSimulatorPlaybackSpeed = (playbackSpeed: number, callback?: () => void) =>
    updateDraft(
      (current) => ({ ...current, simulator: { ...current.simulator, playback_speed: playbackSpeed } }),
      callback,
    )

  const setBagId = (bagId: string, callback?: () => void) =>
    updateDraft((current) => ({ ...current, replay: { ...current.replay, bag_id: bagId } }), callback)

  const setPlayPreprocessed = (playPreprocessed: boolean, callback?: () => void) =>
    updateDraft(
      (current) => ({ ...current, replay: { ...current.replay, play_preprocessed: playPreprocessed } }),
      callback,
    )

  const setDataSource = (nextDataSource: string, callback?: () => void) => {
    setDataSourceState(nextDataSource)
    if (callback) {
      callback()
    }
  }

  const setRuntimeParameters = (forProtocol: string, params: RuntimeParameters, callback?: () => void) =>
    updateDraft(
      (current) => {
        const subject = subjectKey(current.metadata.subject_id)
        return {
          ...current,
          runtimeParametersBySubjectAndProtocol: {
            ...current.runtimeParametersBySubjectAndProtocol,
            [subject]: {
              ...current.runtimeParametersBySubjectAndProtocol[subject],
              [forProtocol]: params,
            },
          },
        }
      },
      callback,
    )

  const buildSessionConfigMessage = (): SessionConfigMessage => ({
    subject_id: draft.metadata.subject_id,
    notes: notes,
    decider_module: draft.pipeline.decider.module,
    decider_enabled: draft.pipeline.decider.enabled,
    preprocessor_module: draft.pipeline.preprocessor.module,
    preprocessor_enabled: draft.pipeline.preprocessor.enabled,
    presenter_module: draft.pipeline.presenter.module,
    presenter_enabled: draft.pipeline.presenter.enabled,
    protocol_filename: protocol,
    runtime_parameters: JSON.stringify(runtimeParameters),
    data_source: dataSource,
    simulator_dataset_filename: draft.simulator.dataset_filename,
    simulator_start_time: draft.simulator.start_time,
    simulator_playback_speed: draft.simulator.playback_speed,
    replay_bag_id: draft.replay.bag_id,
    replay_play_preprocessed: draft.replay.play_preprocessed,
  })

  return (
    <SessionConfigContext.Provider
      value={{
        metadata: draft.metadata,
        pipeline: draft.pipeline,
        simulator: draft.simulator,
        replay: draft.replay,
        dataSource,
        notes,
        getRuntimeParameters,
        isDraftLoaded,
        setSubjectId,
        setNotes,
        setDeciderModule,
        setDeciderEnabled,
        setPreprocessorModule,
        setPreprocessorEnabled,
        setPresenterModule,
        setPresenterEnabled,
        setExperimentProtocol,
        setSimulatorDataset,
        setSimulatorStartTime,
        setSimulatorPlaybackSpeed,
        setBagId,
        setPlayPreprocessed,
        setDataSource,
        setRuntimeParameters,
        buildSessionConfigMessage,
      }}
    >
      {children}
    </SessionConfigContext.Provider>
  )
}

export const useSessionConfig = () => {
  const context = useContext(SessionConfigContext)
  if (context === undefined) {
    throw new Error('useSessionConfig must be used within a SessionConfigProvider')
  }
  return context
}
