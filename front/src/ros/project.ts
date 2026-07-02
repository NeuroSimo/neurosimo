import ROSLIB from '@foxglove/roslibjs'
import { ros } from './ros'

/* Create project service */
const createProjectService = new ROSLIB.Service({
  ros: ros,
  name: '/neurosimo/projects/create',
  serviceType: 'neurosimo_project_interfaces/CreateProject',
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
