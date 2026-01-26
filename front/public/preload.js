const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('electronAPI', {
  openProjectFolder: (projectName, subdir) => ipcRenderer.invoke('open-project-folder', projectName, subdir),
  toggleDetachedExperimentWindow: () => ipcRenderer.invoke('toggle-detached-experiment-window'),
});