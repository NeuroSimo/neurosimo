import os
import threading
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler


class DirectoryWatcherHandler(FileSystemEventHandler):
    """Handler for file system events in watched directories."""
    
    def __init__(self, file_extensions, callback, logger):
        super().__init__()
        self.file_extensions = file_extensions
        self.callback = callback
        self.logger = logger
        self.debounce_timer = None
        self.debounce_delay = 0.5  # 500ms debounce
        self.lock = threading.Lock()
    
    def _should_process_file(self, file_path):
        """Check if the file has one of the watched extensions."""
        return any(file_path.endswith(ext) for ext in self.file_extensions)
    
    def _trigger_callback(self):
        """Trigger the callback after debounce delay."""
        with self.lock:
            if self.debounce_timer:
                self.debounce_timer.cancel()
            self.debounce_timer = threading.Timer(self.debounce_delay, self.callback)
            self.debounce_timer.start()
    
    def on_created(self, event):
        if not event.is_directory and self._should_process_file(event.src_path):
            self.logger.debug(f"File created: {event.src_path}")
            self._trigger_callback()
    
    def on_modified(self, event):
        if not event.is_directory and self._should_process_file(event.src_path):
            self.logger.debug(f"File modified: {event.src_path}")
            self._trigger_callback()
    
    def on_deleted(self, event):
        if not event.is_directory and self._should_process_file(event.src_path):
            self.logger.debug(f"File deleted: {event.src_path}")
            self._trigger_callback()


class DirectoryWatcher:
    """Watches directories for file changes and triggers callbacks."""
    
    def __init__(self, logger):
        self.logger = logger
        self.observer = Observer()
        self.watches = {}  # Maps directory paths to watch handles
        self.observer.start()
    
    def watch_directory(self, directory_path, file_extensions, callback):
        """
        Start watching a directory for files with specific extensions.
        
        Args:
            directory_path: Path to the directory to watch
            file_extensions: List of file extensions to monitor (e.g., ['.py', '.json'])
            callback: Function to call when a relevant file changes
        """
        if not os.path.exists(directory_path):
            self.logger.warning(f"Directory does not exist, cannot watch: {directory_path}")
            return
        
        # Remove existing watch if present
        if directory_path in self.watches:
            self.unwatch_directory(directory_path)
        
        handler = DirectoryWatcherHandler(file_extensions, callback, self.logger)
        watch = self.observer.schedule(handler, directory_path, recursive=False)
        self.watches[directory_path] = watch
        
        self.logger.info(f"Now watching directory: {directory_path} for extensions: {file_extensions}")
    
    def unwatch_directory(self, directory_path):
        """Stop watching a directory."""
        if directory_path in self.watches:
            self.observer.unschedule(self.watches[directory_path])
            del self.watches[directory_path]
            self.logger.debug(f"Stopped watching directory: {directory_path}")
    
    def unwatch_all(self):
        """Stop watching all directories."""
        for directory_path in list(self.watches.keys()):
            self.unwatch_directory(directory_path)
    
    def shutdown(self):
        """Shutdown the directory watcher."""
        self.unwatch_all()
        self.observer.stop()
        self.observer.join()
