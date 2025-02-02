## q3dviewer

`q3dviewer` is a library designed for quickly deploying a 3D viewer. It is based on Qt (PySide6) and provides efficient OpenGL items for displaying 3D objects (e.g., point clouds, cameras, and 3D Gaussians). You can use it to visualize your 3D data or set up an efficient viewer application. It is inspired by PyQtGraph but focuses more on efficient 3D rendering.

## Installation

To install `q3dviewer`, execute the following command in your terminal on either Linux or Windows:

```bash
pip install q3dviewer
```

### Note for Windows Users

- Ensure that you have a Python 3 environment set up:
  - Download and install Python 3 from the [official Python website](https://www.python.org/downloads/).
  - During installation, make sure to check the "Add Python to PATH" option.

### Note for Linux Users

If you encounter an error related to loading the shared library `libxcb-cursor.so.0` on Ubuntu 20.04 or 22.04, please install `libxcb-cursor0`:

```bash
sudo apt-get install libxcb-cursor0
```

## Tools

Once installed, you can directly use the following tools:

### 1. Cloud Viewer

A tool for visualizing point cloud files. Launch it by executing the following command in your terminal:

```sh
cloud_viewer  # The viewer will be displayed
```

*Alternatively*, if the path is not set (though it's not recommended):

```sh
python3 -m q3dviewer.tools.cloud_viewer
```

After the viewer launches, you can drag and drop files onto the window to display the point clouds. Multiple files can be dropped simultaneously to view them together. Supported formats include LAS, PCD, PLY, and E57.

For example, you can download and view point clouds of Tokyo in LAS format from the following link:

[Tokyo Point Clouds](https://www.geospatial.jp/ckan/dataset/tokyopc-23ku-2024/resource/7807d6d1-29f3-4b36-b0c8-f7aa0ea2cff3)

![Cloud Viewer Screenshot](https://qiita-image-store.s3.ap-northeast-1.amazonaws.com/0/149168/03c981c6-1aec-e5b9-4536-e07e1e56ff29.png)

Press `M` on your keyboard to display a menu on the screen, where you can modify visualization settings for each item. For example, you can adjust various settings such as shape, size, color, and transparency for `CloudItem`.

![Cloud Viewer Settings](https://qiita-image-store.s3.ap-northeast-1.amazonaws.com/0/149168/deeb996a-e419-58f4-6bc2-535099b1b73a.png)

### 2. ROS Viewer

A high-performance SLAM viewer compatible with ROS, serving as an alternative to RVIZ.

```sh
roscore &
ros_viewer
```

### 3. Film Maker

Would you like to create a video from point cloud data? With Film Maker, you can easily create videos with simple operations. Just edit keyframes using the user-friendly GUI, and the software will automatically interpolate the keyframes to generate the video.

```sh
film_maker # drag and drop your cloud file to the window
```

* Space key to add a keyframe.
* Delete key to remove a keyframe.

Film Maker GUI: 

![Screenshot from 2025-02-02 18-20-51.png](https://qiita-image-store.s3.ap-northeast-1.amazonaws.com/0/149168/a1a6ad63-237c-482e-439d-e760223c59ca.png)

### 4. Gaussian Viewer

A simple viewer for 3D Gaussians. See [EasyGaussianSplatting](https://github.com/scomup/EasyGaussianSplatting) for more information.

```sh
gaussian_viewer  # Drag and drop your Gaussian file onto the window
```

![Gaussian Viewer GIF](https://qiita-image-store.s3.ap-northeast-1.amazonaws.com/0/149168/441e6f5a-214d-f7c1-11bf-5fa79e63b38e.gif)

### 5. LiDAR-LiDAR Calibration Tools

A tool to compute the relative pose between two LiDARs. It allows for both manual adjustment in the settings screen and automatic calibration.

```sh
lidar_calib --lidar0=/YOUR_LIDAR0_TOPIC --lidar1=/YOUR_LIDAR1_TOPIC
```

![LiDAR Calibration](https://qiita-image-store.s3.ap-northeast-1.amazonaws.com/0/149168/5a8a9903-a42a-8322-1d23-0cbecd3fa99a.png)

### 6. LiDAR-Camera Calibration Tools

A tool for calculating the relative pose between a LiDAR and a camera. It allows for manual adjustment in the settings screen and real-time verification of LiDAR point projection onto images.

```sh
lidar_cam_calib --lidar=/YOUR_LIDAR_TOPIC --camera=/YOUR_CAMERA_TOPIC --camera_info=/YOUR_CAMERA_INFO_TOPIC
```

![LiDAR-Camera Calibration](https://qiita-image-store.s3.ap-northeast-1.amazonaws.com/0/149168/f8359820-2ae7-aa37-6577-0fa035f4dd95.png)

## Using as a Library

Using the examples above, you can easily customize and develop your own 3D viewer with `q3dviewer`. Below is a coding example.

### Custom 3D Viewer

```python
#!/usr/bin/env python3

import q3dviewer as q3d  # Import q3dviewer

def main():
    # Create a Qt application
    app = q3d.QApplication([])

    # Create various 3D items
    axis_item = q3d.AxisItem(size=0.5, width=5)
    grid_item = q3d.GridItem(size=10, spacing=1)

    # Create a viewer
    viewer = q3d.Viewer(name='example')
    
    # Add items to the viewer
    viewer.add_items({
        'grid': grid_item,
        'axis': axis_item,
    })

    # Show the viewer & run the Qt application
    viewer.show()
    app.exec()

if __name__ == '__main__':
    main()
```

`q3dviewer` provides the following 3D items:

- **AxisItem**: Displays coordinate axes or the origin position.
- **CloudItem**: Displays point clouds.
- **CloudIOItem**: Displays point clouds with input/output capabilities.
- **GaussianItem**: Displays 3D Gaussians.
- **GridItem**: Displays grids.
- **ImageItem**: Displays 2D images.
- **Text2DItem**: Displays 2D text.
- **LineItem**: Displays lines or trajectories.

### Developing Custom Items

In addition to the standard 3D items provided, you can visualize custom 3D items with simple coding. Below is a sample:

```python
from OpenGL.GL import *
import numpy as np
import q3dviewer as q3d
from PySide6.QtWidgets import QLabel, QSpinBox

class YourItem(q3d.BaseItem):
    def __init__(self):
        super(YourItem, self).__init__()
        # Necessary initialization

    def add_setting(self, layout):
        # Initialize the settings screen
        label = QLabel("Add your setting:")
        layout.addWidget(label)
        box = QSpinBox()
        layout.addWidget(box)

    def set_data(self, data):
        # Obtain the data you want to visualize
        pass

    def initialize_gl(self):
        # OpenGL initialization settings (if needed)
        pass

    def paint(self):
        # Visualize 3D objects using OpenGL
        pass
```

Enjoy using `q3dviewer`!