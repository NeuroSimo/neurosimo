const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('electronAPI', {
  openProjectFolder: (projectName, subdir) => ipcRenderer.invoke('open-project-folder', projectName, subdir),
});