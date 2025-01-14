import ROSLIB from 'roslib'
import { ros } from './ros'

/* List projects service */
const listProjectsService = new ROSLIB.Service({
  ros: ros,
  name: '/projects/list',
  serviceType: 'project_interfaces/ListProjects',
})

export const listProjects = (callback: (projects: string[]) => void) => {
  const request = new ROSLIB.ServiceRequest({}) as any

  listProjectsService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to list projects: success field was false.')
      } else {
        callback(response.projects)
      }
    },
    (error) => {
      console.log('ERROR: Failed to list projects, error:')
      console.log(error)
    }
  )
}

/* Set active project service */
const setActiveProjectService = new ROSLIB.Service({
  ros: ros,
  name: '/projects/active/set',
  serviceType: 'project_interfaces/SetActiveProject',
})

export const setActiveProject = (project: string, callback: () => void) => {
  const request = new ROSLIB.ServiceRequest({
    project: project,
  }) as any

  setActiveProjectService.callService(
    request,
    (response) => {
      if (!response.success) {
        console.log('ERROR: Failed to set active project: success field was false.')
      } else {
        callback()
      }
    },
    (error) => {
      console.log('ERROR: Failed to set active project, error:')
      console.log(error)
    }
  )
}
