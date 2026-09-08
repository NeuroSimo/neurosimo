import React, { useState, useEffect, ReactNode, createContext, useContext } from 'react'
import { Topic } from '@foxglove/roslibjs'

import { ros } from 'ros/ros'
import { useSystemConfig } from './SystemConfigProvider'

/* Lists the files of one kind within one project. Not to be confused with ProjectList,
   which lists the projects available. */
interface ProjectFileList extends ROSLIB.Message {
  project: string
  filenames: string[]
}

/* The file lists project_watcher publishes for the active project, under the names this
   provider exposes them by. */
const listTopics = {
  preprocessorList: '/neurosimo/pipeline/preprocessor/list',
  deciderList: '/neurosimo/pipeline/decider/list',
  presenterList: '/neurosimo/pipeline/presenter/list',
  protocolList: '/neurosimo/experiment/protocol/list',
  datasetList: '/neurosimo/eeg_simulator/dataset/list',
  externalRecordingsList: '/neurosimo/eeg_simulator/external_recordings/list',
  recordingsList: '/neurosimo/recording/recordings/list',
} as const

type ListName = keyof typeof listTopics

const listNames = Object.keys(listTopics) as ListName[]

type ProjectFiles = Record<ListName, string[]>

/* Stands in until the first project has been received in full. Module-level so that its
   arrays have a stable identity: consumers use them as effect dependencies. */
const emptyProjectFiles: ProjectFiles = {
  preprocessorList: [],
  deciderList: [],
  presenterList: [],
  protocolList: [],
  datasetList: [],
  externalRecordingsList: [],
  recordingsList: [],
}

interface ProjectFilesContextType extends ProjectFiles {
  /* The project the lists describe, which is not necessarily the active one; see below.
     Empty until the first project has been received in full. Everything the UI derives from
     these lists — the selections made against them, the values stored per project — belongs
     to this project rather than to the active one. */
  project: string
}

const defaultProjectFilesState: ProjectFilesContextType = {
  ...emptyProjectFiles,
  project: '',
}

export const ProjectFilesContext = createContext<ProjectFilesContextType>(defaultProjectFilesState)

interface ProjectFilesProviderProps {
  children: ReactNode
}

/* Holds the files of the active project.
 *
 * The lists are published independently of the system configuration that names the active
 * project, so they lag behind a project change by however long project_watcher takes to scan
 * the new project. Each message names the project it describes, so the lag is plain to see.
 *
 * The files of the previously active project are kept on display until every list of the
 * newly active one has arrived, at which point they are swapped in together. Reporting empty
 * lists in the meantime, as would be the obvious thing to do, makes every selection made
 * against them fall back to a placeholder and then back again, which reads as a flicker.
 * project_watcher publishes all of the lists on a project change, so waiting for the whole
 * set terminates.
 */
export const ProjectFilesProvider: React.FC<ProjectFilesProviderProps> = ({ children }) => {
  const { activeProject } = useSystemConfig()

  const [received, setReceived] = useState<Partial<Record<ListName, ProjectFileList>>>({})

  useEffect(() => {
    const subscribers = listNames.map((name) => {
      const subscriber = new Topic<ProjectFileList>({
        ros: ros,
        name: listTopics[name],
        messageType: 'neurosimo_project_interfaces/ProjectFileList',
      })

      subscriber.subscribe((message: ProjectFileList) => {
        setReceived((current) => ({ ...current, [name]: message }))
      })

      return subscriber
    })

    return () => {
      subscribers.forEach((subscriber) => subscriber.unsubscribe())
    }
  }, [])

  /* Whether every list on hand describes the active project. */
  const complete = activeProject !== '' && listNames.every((name) => received[name]?.project === activeProject)

  const [displayed, setDisplayed] = useState<{ project: string; files: ProjectFiles } | null>(null)

  useEffect(() => {
    if (!complete) {
      return
    }

    setDisplayed({
      project: activeProject,
      files: listNames.reduce(
        (files, name) => ({ ...files, [name]: received[name]?.filenames ?? [] }),
        {} as ProjectFiles,
      ),
    })
  }, [complete, received, activeProject])

  return (
    <ProjectFilesContext.Provider
      value={{
        ...(displayed?.files ?? emptyProjectFiles),
        project: displayed?.project ?? '',
      }}
    >
      {children}
    </ProjectFilesContext.Provider>
  )
}

export const useProjectFiles = (): ProjectFilesContextType => useContext(ProjectFilesContext)
