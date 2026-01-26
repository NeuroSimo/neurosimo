const { contextBridge, ipcRenderer } = require('electron');

// Expose protected methods that allow the renderer process to use
// the ipcRenderer without exposing the entire object
contextBridge.exposeInMainWorld('electronAPI', {
  // Example: Add methods here if you need to communicate between
  // main and renderer processes. For now, this is just a placeholder.

  // minimize: () => ipcRenderer.invoke('minimize-window'),
  // maximize: () => ipcRenderer.invoke('maximize-window'),
  // close: () => ipcRenderer.invoke('close-window'),

  // You can add more API methods as needed for your app's functionality
});