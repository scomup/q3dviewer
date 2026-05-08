"""
Setup script for q3dviewer with optional CUDA support
"""
import os
import sys
from pathlib import Path
from setuptools import setup, find_packages, Extension
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    """Custom extension class that doesn't require sources"""

    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CUDAExtension(Extension):
    """Extension for CUDA modules with special build requirements"""
    pass


def find_cuda_path():
    """Find CUDA installation path"""
    # Common CUDA installation paths
    cuda_paths = [
        '/usr/local/cuda',
        '/opt/cuda',
    ]

    # Check environment variable first
    if 'CUDA_HOME' in os.environ:
        cuda_paths.insert(0, os.environ['CUDA_HOME'])
    if 'CUDA_PATH' in os.environ:
        cuda_paths.insert(0, os.environ['CUDA_PATH'])

    # Try nvcc in PATH first
    try:
        import subprocess
        result = subprocess.run(['nvcc', '--version'],
                                capture_output=True,
                                text=True,
                                timeout=5)
        if result.returncode == 0:
            # Try to find CUDA_HOME from nvcc location
            nvcc_path = subprocess.run(['which', 'nvcc'],
                                       capture_output=True,
                                       text=True,
                                       timeout=5)
            if nvcc_path.returncode == 0:
                nvcc_bin = nvcc_path.stdout.strip()
                cuda_home = os.path.dirname(os.path.dirname(nvcc_bin))
                return cuda_home
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    # Check common installation paths
    for cuda_path in cuda_paths:
        nvcc_path = os.path.join(cuda_path, 'bin', 'nvcc')
        if os.path.exists(nvcc_path):
            return cuda_path

    return None


def is_cuda_available():
    """Check if CUDA toolkit is available"""
    return find_cuda_path() is not None


class BuildExtWithCUDA(build_ext):
    """Custom build_ext command that handles CUDA compilation"""

    def run(self):
        """Override run to check for CUDA before building"""
        # Filter out CUDA extensions if CUDA is not available
        cuda_available = is_cuda_available()

        if not cuda_available:
            print("\n" + "="*70)
            print("WARNING: CUDA toolkit not found!")
            print("="*70)
            print("CUDA extensions will be skipped.")
            print("The package will install without GPU acceleration.")
            print("\nTo enable CUDA support:")
            print(
                "  1. Install CUDA Toolkit from https://developer.nvidia.com/cuda-downloads")
            print("  2. Ensure 'nvcc' is in your PATH")
            print("  3. Reinstall: pip install -e .")
            print("="*70 + "\n")

            # Remove CUDA extensions from the build list
            self.extensions = [ext for ext in self.extensions
                               if not isinstance(ext, CUDAExtension)]
        else:
            print("\n" + "="*70)
            print("CUDA toolkit detected - building GPU extensions")
            print("="*70 + "\n")

        # Continue with normal build
        if self.extensions:
            build_ext.run(self)

    def build_extension(self, ext):
        """Build a single extension"""
        if isinstance(ext, CUDAExtension):
            self.build_cuda_extension(ext)
        else:
            build_ext.build_extension(self, ext)

    def build_cuda_extension(self, ext):
        """Build CUDA extension using nvcc"""
        import subprocess
        import numpy as np

        # Find CUDA installation
        cuda_home = find_cuda_path()
        if cuda_home is None:
            raise RuntimeError("CUDA installation not found")

        nvcc = os.path.join(cuda_home, 'bin', 'nvcc')
        if not os.path.exists(nvcc):
            raise RuntimeError(f"nvcc not found at {nvcc}")

        print(f"Using CUDA from: {cuda_home}")
        print(f"Using nvcc: {nvcc}")

        extdir = os.path.abspath(os.path.dirname(
            self.get_ext_fullpath(ext.name)))

        # Ensure the output directory exists
        os.makedirs(extdir, exist_ok=True)

        # Find pybind11 headers
        try:
            import pybind11
            pybind11_include = pybind11.get_include()
        except ImportError:
            raise RuntimeError("pybind11 is required to build CUDA extensions. "
                               "Install it with: pip install pybind11")

        # Python and numpy include paths
        python_include = self.distribution.get_command_obj(
            'build').build_platlib
        if hasattr(sys, 'base_prefix'):
            python_include = os.path.join(sys.base_prefix, 'include',
                                          f'python{sys.version_info.major}.{sys.version_info.minor}')
        else:
            python_include = os.path.join(sys.prefix, 'include',
                                          f'python{sys.version_info.major}.{sys.version_info.minor}')

        numpy_include = np.get_include()
        cuda_include = os.path.join(cuda_home, 'include')

        # Output shared library path
        output_name = self.get_ext_filename(ext.name)
        output_path = os.path.join(extdir, os.path.basename(output_name))

        # CUDA compilation flags
        cuda_flags = [
            '-O3',
            '--compiler-options', '-fPIC',
            '-std=c++14',
            '--expt-relaxed-constexpr',
            '-DNDEBUG',
        ]

        # Add GPU architectures (compute capabilities)
        # Cover common GPUs from Pascal to Ada Lovelace
        cuda_flags.extend([
            '-gencode=arch=compute_60,code=sm_60',  # Pascal (GTX 10 series)
            '-gencode=arch=compute_61,code=sm_61',  # Pascal
            '-gencode=arch=compute_70,code=sm_70',  # Volta (V100)
            '-gencode=arch=compute_75,code=sm_75',  # Turing (RTX 20 series)
            # Ampere (RTX 30 series, A100)
            '-gencode=arch=compute_80,code=sm_80',
            '-gencode=arch=compute_86,code=sm_86',  # Ampere (RTX 30 series)
            # Ada Lovelace (RTX 40 series)
            '-gencode=arch=compute_89,code=sm_89',
        ])

        # Include paths
        include_flags = [
            f'-I{python_include}',
            f'-I{numpy_include}',
            f'-I{pybind11_include}',
            f'-I{cuda_include}',
        ]

        # Build command
        cmd = [nvcc, '--shared'] + cuda_flags + include_flags

        # Add source files
        for source in ext.sources:
            cmd.append(source)

        # Output file
        cmd.extend(['-o', output_path])

        # Link against Python
        if sys.platform == 'darwin':
            # macOS
            cmd.extend(['-undefined', 'dynamic_lookup'])
        else:
            # Linux
            python_lib = f'python{sys.version_info.major}.{sys.version_info.minor}'
            cmd.extend([f'-l{python_lib}'])

        print(f"Building CUDA extension: {ext.name}")
        print(f"Command: {' '.join(cmd)}")

        try:
            subprocess.check_call(cmd)
            print(f"Successfully built {ext.name}")
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Failed to build CUDA extension: {e}")


def get_extensions():
    """Define extensions to build"""
    extensions = []

    # CUDA extension (optional)
    cuda_source = 'q3dviewer/libs/cuda_point_sort.cu'
    if os.path.exists(cuda_source):
        extensions.append(
            CUDAExtension(
                'q3dviewer.cuda_point_sort',
                sources=[cuda_source],
            )
        )

    return extensions


# Minimal setup.py for dynamic CUDA extension building
# All static metadata is defined in pyproject.toml
setup(
    ext_modules=get_extensions(),
    cmdclass={
        'build_ext': BuildExtWithCUDA,
    },
)
