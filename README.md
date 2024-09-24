
## q3dviewer

q3dviewer is a library used for quickly deploying a 3D viewer. It is based on pyqtgraph and provides more efficient widgets for displaying 3D objects (e.g., point clouds, cameras, 3D Gaussian). You can use it to visualize your 3D data or set up an efficient viewer application.

### Install

```console
cd q3dviewer
pip install -e .
```

### Example

#### Cloud Viewer:

```console
cloud_viewer --pcd="YOUR_PCD_FILE_PATH"
```
You also can drag your file the main window of cloud viewer.


#### Ros Viewer:

viewer ros cloud topic.
```console
roscore&
ros_viewer
```

