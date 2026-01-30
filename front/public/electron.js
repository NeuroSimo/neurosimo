const { app, BrowserWindow, ipcMain, shell } = require('electron');
const path = require('path');
const os = require('os');
const isDev = process.env.NODE_ENV === 'development';

let mainWindow;
let detachedWindow = null;

function createWindow() {
  // Create the browser window.
  mainWindow = new BrowserWindow({
    width: 1200,
    height: 800,
    minWidth: 800,
    minHeight: 600,
    fullscreen: true,
    webPreferences: {
      nodeIntegration: false,
      contextIsolation: true,
      enableRemoteModule: false,
      preload: path.join(__dirname, 'preload.js')
    },
    icon: path.join(__dirname, 'favicon.ico'),
    show: false, // Don't show until ready-to-show
  });

  // Load the app
  if (isDev) {
    mainWindow.loadURL('http://localhost:3000');
    // Open the DevTools in development
    mainWindow.webContents.openDevTools();
  } else {
    mainWindow.loadFile(path.join(__dirname, '../build/index.html'));
  }

  // Show window when ready to prevent visual flash
  mainWindow.once('ready-to-show', () => {
    mainWindow.show();
  });

  // Emitted when the window is closed.
  mainWindow.on('closed', () => {
    mainWindow = null;
  });
}

function toggleDetachedWindow() {
  if (detachedWindow) {
    if (detachedWindow.isVisible()) {
      detachedWindow.hide();
    } else {
      detachedWindow.show();
      detachedWindow.focus();
    }
    return;
  }

  detachedWindow = new BrowserWindow({
    width: 800,
    height: 600,
    backgroundColor: '#000000',
    fullscreenable: true,
    frame: false,
    autoHideMenuBar: true,
    webPreferences: {
      nodeIntegration: false,
      contextIsolation: true,
      enableRemoteModule: false,
      preload: path.join(__dirname, 'preload.js')
    }
  });

  if (isDev) {
    detachedWindow.loadURL('http://localhost:3000?detached=true');
  } else {
    detachedWindow.loadFile(path.join(__dirname, '../build/index.html'), {
      query: { detached: 'true' }
    });
  }

  detachedWindow.on('closed', () => {
    detachedWindow = null;
  });
}

// Handle IPC calls
ipcMain.handle('open-project-folder', async (event, projectName, subdir) => {
  const folderPath = path.join(os.homedir(), 'projects', projectName, subdir);
  return await shell.openPath(folderPath);
});

ipcMain.handle('toggle-detached-experiment-window', async () => {
  toggleDetachedWindow();
  return null;
});

// This method will be called when Electron has finished initialization
app.whenReady().then(createWindow);

// Quit when all windows are closed.
app.on('window-all-closed', () => {
  // On macOS it is common for applications and their menu bar
  // to stay active until the user quits explicitly with Cmd + Q
  if (process.platform !== 'darwin') {
    app.quit();
  }
});

app.on('activate', () => {
  // On macOS it's common to re-create a window in the app when the
  // dock icon is clicked and there are no other windows open.
  if (BrowserWindow.getAllWindows().length === 0) {
    createWindow();
  }
});

// Security: Prevent new window creation
app.on('web-contents-created', (event, contents) => {
  contents.on('new-window', (event, navigationUrl) => {
    event.preventDefault();
    require('electron').shell.openExternal(navigationUrl);
  });
});