from setuptools import setup, find_packages

setup(
    name='fast_viewer',
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
            'cloud_viewer = fast_viewer.cloud_viewer:main',
            'ros_viewer = fast_viewer.ros_viewer:main',
            'mesh_viewer = fast_viewer.mesh_viewer:main',
            'gaussian_viewer = fast_viewer.gaussian_viewer:main',
        ],
    },
)