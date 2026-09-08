import { useEffect, useState } from 'react'
import { Topic } from '@foxglove/roslibjs'

import { ros } from 'ros/ros'

interface ProjectFileList extends ROSLIB.Message {
  project: string
  filenames: string[]
}

/* Returned when the list for the active project is not known yet. Module-level so that its
   identity is stable: it is used as an effect dependency by the consumers. */
const emptyList: string[] = []

/* Subscribes to one of the project file list topics published by project_watcher.
 *
 * These lists are published independently of the system configuration that determines the
 * active project, so the list on hand does not necessarily describe the project the UI
 * considers active: after a project change the previous project's list is held until
 * project_watcher has scanned the new one, and on startup the two latched topics arrive in
 * either order. Each message names its project, so the mismatch is simply detected.
 *
 * Reports an empty list while the project does not match, which the callers render as a
 * placeholder. The alternative, showing the list of the project just left, would have the
 * UI judge the new project's selections against the wrong set of files.
 */
export const useProjectFileList = (topicName: string, activeProject: string): string[] => {
  const [received, setReceived] = useState<ProjectFileList | null>(null)

  useEffect(() => {
    const subscriber = new Topic<ProjectFileList>({
      ros: ros,
      name: topicName,
      messageType: 'neurosimo_project_interfaces/ProjectFileList',
    })

    subscriber.subscribe((message: ProjectFileList) => {
      setReceived(message)
    })

    return () => {
      subscriber.unsubscribe()
    }
  }, [topicName])

  if (received === null || received.project !== activeProject) {
    return emptyList
  }
  return received.filenames
}
