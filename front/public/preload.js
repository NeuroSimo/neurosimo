const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('electronAPI', {
  openRecordingsFolder: (projectName) => ipcRenderer.invoke('open-recordings-folder', projectName),
});