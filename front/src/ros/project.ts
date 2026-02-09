import ROSLIB from '@foxglove/roslibjs'
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

/* Create project service */
const createProjectService = new ROSLIB.Service({
  ros: ros,
  name: '/projects/create',
  serviceType: 'project_interfaces/CreateProject',
})

export const createProject = (projectName: string, callback: (success: boolean) => void) => {
  const request = new ROSLIB.ServiceRequest({
    project_name: projectName,
  }) as any

  createProjectService.callService(
    request,
    (response) => {
      callback(response.success)
    },
    (error) => {
      console.log('ERROR: Failed to create project, error:')
      console.log(error)
      callback(false)
    }
  )
}
