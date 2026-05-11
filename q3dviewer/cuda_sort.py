"""
CUDA-OpenGL Interop for Point Cloud Depth Sorting
==================================================

Python wrapper for CUDA-accelerated point cloud sorting.
This module provides a clean interface to the C++ CUDA implementation.

Architecture:
    - CUDA kernels (cuda_point_sort.cu) - GPU computation
    - Pybind11 bindings - C++/Python interface  
    - This module - High-level Python API

Usage:
    >>> sorter = CUDAPointSorter()
    >>> if sorter.is_available():
    >>>     sorter.register(points_ssbo, max_points=100000)
    >>>     sorter.sort_by_depth(depth_coeffs, n_points)
    >>>     sorter.unregister()
"""

import numpy as np
from typing import Optional

__all__ = ['CUDAPointSorter', 'is_cuda_available']


# Try to import the C++ extension
try:
    from q3dviewer.cuda_point_sort import (
        CUDAPointSorter as _CUDAPointSorterImpl,
        is_cuda_available as _is_cuda_available_impl
    )
    _CUDA_AVAILABLE = True
except ImportError as e:
    _CUDA_AVAILABLE = False
    _import_error = str(e)


class CUDAPointSorter:
    """
    High-level Python interface for CUDA-accelerated point cloud sorting.

    This class provides depth-based sorting of point clouds using GPU acceleration
    with OpenGL-CUDA interop for zero-copy operation.

    Workflow:
        1. register() - Register OpenGL SSBO with CUDA
        2. sort_by_depth() - Sort points by depth (can be called multiple times)
        3. unregister() - Clean up resources

    Example:
        >>> sorter = CUDAPointSorter()
        >>> if sorter.is_available():
        >>>     sorter.register(points_ssbo_id, max_points=100000)
        >>>     
        >>>     # In render loop
        >>>     depth_rot = view_matrix[2, :3]  # z-row of view matrix
        >>>     sorter.sort_by_depth(depth_rot, num_points)
        >>>     
        >>>     # On cleanup
        >>>     sorter.unregister()
    """

    def __init__(self):
        """
        Initialize CUDA point sorter.

        If CUDA is not available, the sorter will be in a disabled state.
        """
        self._impl = None

        if _CUDA_AVAILABLE:
            try:
                self._impl = _CUDAPointSorterImpl()
            except Exception as e:
                print(f"[CUDA] Failed to create sorter: {e}")
                self._impl = None

    def is_available(self) -> bool:
        """
        Check if CUDA sorting is available.

        Returns:
            bool: True if CUDA is available and working
        """
        return self._impl is not None

    def register(self, points_ssbo_id: int, max_points: int):
        """
        Register OpenGL SSBO with CUDA for interop.

        This establishes the connection between OpenGL and CUDA, allowing
        CUDA to directly access the point cloud data stored in GPU memory.

        Args:
            points_ssbo_id: OpenGL buffer ID (GLuint) for points SSBO
            max_points: Maximum number of points (for buffer allocation)

        Raises:
            RuntimeError: If CUDA is not available or registration fails
        """
        if not self.is_available():
            raise RuntimeError(
                "CUDA is not available. Cannot register buffers."
                f"\nReason: {_import_error if not _CUDA_AVAILABLE else 'Unknown'}"
            )

        self._impl.register_buffers(points_ssbo_id, max_points)

    def sort_by_depth(self, depth_rotation: np.ndarray, num_points: int):
        """
        Sort points by depth and reorder point data in-place.

        Computes depth as: depth = a*x + b*y + c*z
        where [a, b, c] are the rotation coefficients (typically the z-row
        of the combined view*model matrix).

        Points are sorted far-to-near for correct transparency blending.

        Args:
            depth_rotation: 3-element numpy array [a, b, c] (float32)
                           Typically view_matrix[2, :3] or combined[2, :3]
            num_points: Number of points to sort

        Raises:
            RuntimeError: If buffers not registered or sorting fails
        """
        if not self.is_available():
            raise RuntimeError("CUDA is not available")

        # Ensure depth_rotation is the correct type
        depth_coeffs = np.ascontiguousarray(depth_rotation, dtype=np.float32)
        if depth_coeffs.size != 3:
            raise ValueError(
                f"depth_rotation must have 3 elements, got {depth_coeffs.size}")

        self._impl.sort_by_depth(depth_coeffs, num_points)

    def unregister(self):
        """
        Unregister CUDA resources and clean up.

        This should be called when done with the sorter, or when the
        OpenGL context is being destroyed.
        """
        if self._impl is not None:
            self._impl.unregister()

    def is_registered(self) -> bool:
        """
        Check if buffers are currently registered.

        Returns:
            bool: True if buffers are registered
        """
        if not self.is_available():
            return False
        return self._impl.is_registered()

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.unregister()


def is_cuda_available() -> bool:
    """
    Check if CUDA sorting is available on this system.

    Returns:
        bool: True if CUDA library is loaded and working
    """
    if not _CUDA_AVAILABLE:
        return False

    try:
        return _is_cuda_available_impl()
    except:
        return False


if __name__ == '__main__':
    # Test CUDA availability
    print("="*70)
    print("CUDA Point Sorting - Availability Check")
    print("="*70)
    print()

    if is_cuda_available():
        print("✓ CUDA library loaded successfully")
        print()
        print("Features:")
        print("  - Direct SSBO access (zero-copy GPU operation)")
        print("  - Automatic depth calculation from positions")
        print("  - Thrust-based GPU sorting")
        print("  - Pybind11 Python bindings")
        print()

        # Try to create a sorter
        try:
            sorter = CUDAPointSorter()
            if sorter.is_available():
                print("✓ CUDAPointSorter created successfully")
            else:
                print("✗ Failed to create CUDAPointSorter")
        except Exception as e:
            print(f"✗ Error creating sorter: {e}")
    else:
        print("✗ CUDA library not available")
        print()
        print("To enable CUDA sorting:")
        print("  1. Install CUDA Toolkit from https://developer.nvidia.com/cuda-downloads")
        print("  2. Run: pip install -e . (to rebuild with CUDA support)")
        print()
        if not _CUDA_AVAILABLE:
            print(f"Import error: {_import_error}")

    print("="*70)
