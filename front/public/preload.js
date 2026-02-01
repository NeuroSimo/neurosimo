const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('electronAPI', {
  openProjectFolder: (projectName, subdir) => ipcRenderer.invoke('open-project-folder', projectName, subdir),
  openTerminalInFolder: (projectName, subdir) => ipcRenderer.invoke('open-terminal-in-folder', projectName, subdir),
  toggleDetachedExperimentWindow: () => ipcRenderer.invoke('toggle-detached-experiment-window'),
});