import numpy as np
from typing import Optional

__all__ = ['PointSorter', 'CUDAPointSorter', 'is_cuda_available']


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


class PointSorter:
    """
    High-level Python interface for CUDA-accelerated point cloud sorting.

    This class provides depth-based sorting of point clouds using GPU acceleration
    with OpenGL-CUDA interop for zero-copy operation. Falls back to CPU sorting
    when CUDA is not available.

    Workflow:
        1. register() - Register OpenGL SSBO with CUDA (or prepare for CPU fallback)
        2. sort_by_depth() - Sort points by depth (can be called multiple times)
        3. unregister() - Clean up resources

    Example:
        >>> sorter = PointSorter()
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

    # Point data structure (matches CloudItem.DATA_TYPE)
    STRIDE = 16  # bytes per point
    DATA_TYPE = np.dtype([('xyz', '<f4', (3,)), ('irgb', '<u4')])

    def __init__(self):
        """
        Initialize point sorter.

        If CUDA is not available, falls back to CPU sorting.
        """
        # CUDA implementation (None = use CPU fallback)
        self._impl = None

        # CPU fallback state
        self._vbo_id = None
        self._max_points = 0

        # Try to initialize CUDA implementation
        if _CUDA_AVAILABLE:
            try:
                self._impl = _CUDAPointSorterImpl()
            except Exception as e:
                print(
                    f"[PointSorter] CUDA initialization failed, using CPU fallback: {e}")

    def is_using_cuda(self) -> bool:
        """
        Check if CUDA GPU sorting is being used.

        Returns:
            bool: True if using CUDA, False if using CPU fallback
        """
        return self._impl is not None

    def register(self, points_ssbo_id: int, max_points: int):
        """
        Register OpenGL SSBO for sorting (CUDA or CPU).

        This establishes the connection between OpenGL and the sorting backend.
        For CUDA, it enables direct access to GPU buffer. For CPU, it stores
        the VBO ID for later download/upload operations.

        Args:
            points_ssbo_id: OpenGL buffer ID (GLuint) for points SSBO
            max_points: Maximum number of points (for buffer allocation)

        Raises:
            RuntimeError: If registration fails
        """
        if self._impl is not None:
            # CUDA path: register for zero-copy GPU access
            try:
                self._impl.register_buffers(points_ssbo_id, max_points)
            except RuntimeError as e:
                if "out of memory" in str(e).lower():
                    print(
                        f"[PointSorter] CUDA out of memory for {max_points} points, "
                        f"falling back to CPU sorting: {e}")
                    self._impl = None
                    self._vbo_id = points_ssbo_id
                    self._max_points = max_points
                else:
                    raise
        else:
            # CPU fallback: just store VBO ID for later use
            self._vbo_id = points_ssbo_id
            self._max_points = max_points

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
        # Ensure depth_rotation is the correct type
        depth_coeffs = np.ascontiguousarray(depth_rotation, dtype=np.float32)
        if depth_coeffs.size != 3:
            raise ValueError(
                f"depth_rotation must have 3 elements, got {depth_coeffs.size}")

        if self._impl is not None:
            # CUDA path: fast GPU sorting
            self._impl.sort_by_depth(depth_coeffs, num_points)
        else:
            # CPU fallback: download, sort, upload
            self._cpu_sort_by_depth(depth_coeffs, num_points)

    def _cpu_sort_by_depth(self, depth_coeffs: np.ndarray, num_points: int):
        """
        CPU fallback for depth sorting.

        Downloads point data from GPU, sorts on CPU, and uploads back.
        This is slower than CUDA but provides depth sorting when CUDA is unavailable.

        Args:
            depth_coeffs: 3-element array [a, b, c] for depth = a*x + b*y + c*z
            num_points: Number of points to sort
        """
        from OpenGL.GL import (
            glBindBuffer, glGetBufferSubData, glBufferSubData,
            GL_ARRAY_BUFFER
        )

        if self._vbo_id is None:
            raise RuntimeError(
                "Buffers not registered. Call register() first.")

        # Download data from GPU to CPU
        glBindBuffer(GL_ARRAY_BUFFER, self._vbo_id)
        buffer_data = glGetBufferSubData(
            GL_ARRAY_BUFFER,
            0,
            num_points * self.STRIDE
        )
        glBindBuffer(GL_ARRAY_BUFFER, 0)

        # Convert to numpy structured array
        points = np.frombuffer(
            buffer_data, dtype=self.DATA_TYPE, count=num_points)

        # Calculate depth for each point
        xyz = points['xyz']
        depths = xyz @ depth_coeffs  # depth = a*x + b*y + c*z

        # Sort indices by depth (ascending order, same as GPU)
        # Ascending order to match GPU CUB sort
        sort_indices = np.argsort(depths)

        # Reorder points
        sorted_points = points[sort_indices]

        # Upload sorted data back to GPU
        glBindBuffer(GL_ARRAY_BUFFER, self._vbo_id)
        glBufferSubData(
            GL_ARRAY_BUFFER,
            0,
            num_points * self.STRIDE,
            sorted_points
        )
        glBindBuffer(GL_ARRAY_BUFFER, 0)

    def unregister(self):
        """
        Unregister resources and clean up.

        This should be called when done with the sorter, or when the
        OpenGL context is being destroyed.
        """
        if self._impl is not None:
            self._impl.unregister()

        # CPU fallback: just clear state
        self._vbo_id = None
        self._max_points = 0

    def is_registered(self) -> bool:
        """
        Check if buffers are currently registered.

        Returns:
            bool: True if buffers are registered
        """
        if self._impl is not None:
            return self._impl.is_registered()
        else:
            # CPU fallback: check if VBO ID is set
            return self._vbo_id is not None

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
