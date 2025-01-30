from setuptools import setup, find_packages

setup(
    name='q3dviewer',
    version='1.0.8',
    packages=find_packages(),
    include_package_data=True,
    package_data={
        'q3dviewer': ['shaders/*.glsl'],
    },
    install_requires=[
        'numpy',
        'pyside6',
        'PyOpenGL',
        'pillow',
        'meshio',
        'pypcd4',
        'pye57',
        'laspy',
        ],
    entry_points={
        'console_scripts': [
            'cloud_viewer = q3dviewer.tools.cloud_viewer:main',
            'ros_viewer = q3dviewer.tools.ros_viewer:main',
            'mesh_viewer = q3dviewer.tools.mesh_viewer:main',
            'gaussian_viewer = q3dviewer.tools.gaussian_viewer:main',
            'lidar_cam_calib = q3dviewer.tools.lidar_cam_calib:main',
            'lidar_calib = q3dviewer.tools.lidar_calib:main',
        ],
    },
)