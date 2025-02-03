from setuptools import setup, find_packages

setup(
    name='q3dviewer',
    version='1.1.1',
    author="Liu Yang",
    description="A library designed for quickly deploying a 3D viewer.",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/scomup/q3dviewer",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
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
        'imageio',
        'imageio[ffmpeg]',
    ],
    entry_points={
        'console_scripts': [
            'cloud_viewer = q3dviewer.tools.cloud_viewer:main',
            'ros_viewer = q3dviewer.tools.ros_viewer:main',
            'mesh_viewer = q3dviewer.tools.mesh_viewer:main',
            'gaussian_viewer = q3dviewer.tools.gaussian_viewer:main',
            'lidar_cam_calib = q3dviewer.tools.lidar_cam_calib:main',
            'lidar_calib = q3dviewer.tools.lidar_calib:main',
            'film_maker = q3dviewer.tools.film_maker:main',
        ],
    },
)