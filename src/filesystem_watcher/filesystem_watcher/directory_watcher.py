import os
import threading
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler


class DirectoryWatcherHandler(FileSystemEventHandler):
    """Handler for file system events in watched directories.

    Operates in one of two modes:
      - file mode (default): reacts to files whose name ends with one of
        ``file_extensions`` being created/modified/deleted.
      - directory mode (``watch_directories=True``): reacts to immediate
        subdirectories being created/deleted/renamed. Content modifications are
        ignored so that changes to files *inside* a subdirectory do not trigger
        the callback.
    """

    def __init__(self, callback, logger, file_extensions=None, watch_directories=False):
        super().__init__()
        self.file_extensions = file_extensions or []
        self.callback = callback
        self.logger = logger
        self.watch_directories = watch_directories
        self.debounce_timer = None
        self.debounce_delay = 0.5  # 500ms debounce
        self.lock = threading.Lock()

    def _has_watched_extension(self, file_path):
        """Check if the file has one of the watched extensions."""
        return any(file_path.endswith(ext) for ext in self.file_extensions)

    def _should_process(self, event):
        """Decide whether an event is relevant for the current mode."""
        if self.watch_directories:
            return event.is_directory
        return not event.is_directory and self._has_watched_extension(event.src_path)

    def _trigger_callback(self):
        """Trigger the callback after debounce delay."""
        with self.lock:
            if self.debounce_timer:
                self.debounce_timer.cancel()
            self.debounce_timer = threading.Timer(self.debounce_delay, self.callback)
            self.debounce_timer.start()

    def on_created(self, event):
        if self._should_process(event):
            self.logger.debug(f"Created: {event.src_path}")
            self._trigger_callback()

    def on_modified(self, event):
        # In directory mode a directory's mtime changes whenever its contents
        # change; ignore those so only add/remove/rename of subdirectories count.
        if self.watch_directories:
            return
        if self._should_process(event):
            self.logger.debug(f"Modified: {event.src_path}")
            self._trigger_callback()

    def on_deleted(self, event):
        if self._should_process(event):
            self.logger.debug(f"Deleted: {event.src_path}")
            self._trigger_callback()

    def on_moved(self, event):
        # A rename fires a move event with the old path as src and the new path
        # as dest. Both need to be considered so that renaming a file into or out
        # of the watched set updates the listing.
        if self.watch_directories:
            if event.is_directory:
                self.logger.debug(f"Moved: {event.src_path} -> {event.dest_path}")
                self._trigger_callback()
            return
        if event.is_directory:
            return
        if self._has_watched_extension(event.src_path) or self._has_watched_extension(event.dest_path):
            self.logger.debug(f"Moved: {event.src_path} -> {event.dest_path}")
            self._trigger_callback()


class DirectoryWatcher:
    """Watches directories for changes and triggers callbacks."""

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
        handler = DirectoryWatcherHandler(
            callback=callback,
            logger=self.logger,
            file_extensions=file_extensions,
        )
        self._schedule(directory_path, handler, description=f"extensions: {file_extensions}")

    def watch_subdirectories(self, directory_path, callback):
        """
        Start watching a directory for immediate subdirectories being
        added, removed or renamed.

        Args:
            directory_path: Path to the directory whose subdirectories to monitor
            callback: Function to call when the set of subdirectories changes
        """
        handler = DirectoryWatcherHandler(
            callback=callback,
            logger=self.logger,
            watch_directories=True,
        )
        self._schedule(directory_path, handler, description="subdirectories")

    def _schedule(self, directory_path, handler, description):
        """Schedule a watch, replacing any existing watch on the same path."""
        if not os.path.exists(directory_path):
            self.logger.warning(f"Directory does not exist, cannot watch: {directory_path}")
            return

        # Remove existing watch if present
        if directory_path in self.watches:
            self.unwatch_directory(directory_path)

        watch = self.observer.schedule(handler, directory_path, recursive=False)
        self.watches[directory_path] = watch

        self.logger.info(f"Now watching directory: {directory_path} for {description}")

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
