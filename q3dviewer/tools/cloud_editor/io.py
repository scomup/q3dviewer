"""File input and output for the cloud editor."""

from pathlib import Path

from q3dviewer.Qt.QtCore import QObject, QThread, Signal


def _load_cloud_file(file_path):
    suffix = Path(file_path).suffix.lower()
    loaders = {
        '.pcd': 'load_pcd',
        '.ply': 'load_ply',
        '.e57': 'load_e57',
        '.las': 'load_las',
    }
    loader_name = loaders.get(suffix)
    if loader_name is None:
        return None
    from q3dviewer.utils import cloud_io
    return getattr(cloud_io, loader_name)(str(file_path))


class FileLoaderThread(QThread):
    progress = Signal(int, int, str)
    cloud_loaded = Signal(object, bool)
    load_error = Signal(str)

    def __init__(self, files):
        super().__init__()
        self.files = files

    def run(self):
        for index, file_path in enumerate(self.files):
            self.progress.emit(index + 1, len(self.files), file_path.name)
            try:
                cloud = _load_cloud_file(file_path)
            except Exception as error:
                self.load_error.emit(f"Cannot load {file_path}\n{error}")
                continue
            if cloud is not None:
                self.cloud_loaded.emit(cloud, index > 0)


class CloudEditorIO(QObject):
    """Load and save clouds without knowing how they are edited or displayed."""

    cloud_loaded = Signal(object, bool)
    progress = Signal(int, int, str)
    loading_finished = Signal()
    operation_error = Signal(str)
    save_finished = Signal(str)

    def __init__(self):
        super().__init__()
        self._load_thread = None

    def load_file(self, file, append=False):
        file_path = Path(file)
        try:
            cloud = _load_cloud_file(file_path)
        except Exception as error:
            self.operation_error.emit(f"Cannot load {file_path}\n{error}")
            return
        if cloud is not None:
            self.cloud_loaded.emit(cloud, append)

    def load_files(self, files):
        if self._load_thread is not None and self._load_thread.isRunning():
            return
        file_paths = [Path(file) for file in files]
        if not file_paths:
            self.loading_finished.emit()
            return

        self._load_thread = FileLoaderThread(file_paths)
        self._load_thread.progress.connect(self.progress)
        self._load_thread.cloud_loaded.connect(self.cloud_loaded)
        self._load_thread.load_error.connect(self.operation_error)
        self._load_thread.finished.connect(self._loading_finished)
        self._load_thread.finished.connect(self._load_thread.deleteLater)
        self._load_thread.start()

    def _loading_finished(self):
        self._load_thread = None
        self.loading_finished.emit()

    def save(self, cloud, file):
        if cloud is None:
            return
        path = Path(file)
        try:
            if path.suffix.lower() == '.pcd':
                from q3dviewer.utils.cloud_io import save_pcd as func
            elif path.suffix.lower() == '.ply':
                from q3dviewer.utils.cloud_io import save_ply as func
            elif path.suffix.lower() == '.las':
                from q3dviewer.utils.cloud_io import save_las as func
            elif path.suffix.lower() == '.e57':
                from q3dviewer.utils.cloud_io import save_e57 as func
            else:
                self.operation_error.emit(
                    f"Unsupported file type: {path.suffix}")
                return
            func(cloud, str(path))
        except Exception as error:
            self.operation_error.emit(f"Cannot save to {path}\n{error}")
            return
        self.save_finished.emit(str(path))
