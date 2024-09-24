from setuptools import setup, find_packages

setup(
    name='q3dviewer',
    version='1.0.0',
    packages=find_packages(),
    install_requires=[
        'numpy',
        'pyqtgraph',
        'pyqt5',
        'PyOpenGL',
        'pypcd4',
        'pillow'
        ],
    entry_points={
        'console_scripts': [
            'cloud_viewer = q3dviewer.cloud_viewer:main',
            'ros_viewer = q3dviewer.ros_viewer:main',
            'mesh_viewer = q3dviewer.mesh_viewer:main',
            'gaussian_viewer = q3dviewer.gaussian_viewer:main',
        ],
    },
)