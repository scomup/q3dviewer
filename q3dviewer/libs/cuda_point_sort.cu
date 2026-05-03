/*
 * cuda_point_sort.cu
 * ------------------
 * CUDA-GL interop for depth-based point cloud sorting.
 * 
 * Takes point positions from OpenGL SSBO, calculates depth values,
 * and sorts indices by depth - all on GPU without CPU transfer.
 */

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <cub/cub.cuh>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <cstdio>
#include <cstddef>
#include <stdexcept>
#include <memory>

namespace py = pybind11;

// Point data structure matching Python/OpenGL layout
struct PointData {
    float xyz[3];
    unsigned int irgb;
};

// CUDA error checking macro that throws exceptions
#define CUDA_CHECK(call) \
    do { \
        cudaError_t err = (call); \
        if (err != cudaSuccess) { \
            char error_msg[256]; \
            snprintf(error_msg, sizeof(error_msg), \
                     "[CUDA] Error at %s:%d - %s", \
                     __FILE__, __LINE__, cudaGetErrorString(err)); \
            throw std::runtime_error(error_msg); \
        } \
    } while(0)

// CUDA kernel to calculate depth from positions
// Coefficients passed as immediate values (not pointer) for better performance
__global__ void compute_depths_kernel(
    const PointData* points,
    float coeff_a, float coeff_b, float coeff_c,  // Rotation coefficients for depth = ax + by + cz
    float* depths,
    unsigned int* indices,
    unsigned int num_points)
{
    unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;
    
    // Get point position
    float x = points[idx].xyz[0];
    float y = points[idx].xyz[1];
    float z = points[idx].xyz[2];
    
    // Compute depth using coefficients (passed via constant cache)
    float depth = coeff_a * x + coeff_b * y + coeff_c * z;
    
    depths[idx] = depth;
    indices[idx] = idx;
}

// Custom gather kernel for point reordering
__global__ void gather_points_kernel(
    const unsigned int* indices,
    const PointData* points_in,
    PointData* points_out,
    unsigned int num_points)
{
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= num_points) return;
    
    // Get source index from sorted indices
    unsigned int src_idx = indices[i];
    
    // Copy point data from source to destination
    points_out[i] = points_in[src_idx];
}


//=============================================================================
// C++ Class Interface for Python Binding
//=============================================================================

/**
 * CUDA Point Sorter for OpenGL-CUDA Interop
 * 
 * Provides depth-based sorting of point clouds directly on GPU.
 */
class CUDAPointSorter {
private:
    cudaGraphicsResource_t points_resource_;
    float* d_depths_;           // Input depth values for CUB sort
    float* d_depths_out_;       // Output depth values from CUB sort
    unsigned int* d_indices_;   // Input indices for CUB sort
    unsigned int* d_indices_out_; // Output indices from CUB sort
    PointData* d_points_temp_;
    void* d_cub_temp_storage_;  // CUB temporary storage
    size_t cub_temp_storage_bytes_;
    size_t max_points_;
    bool registered_;

public:
    /**
     * Constructor
     */
    CUDAPointSorter() 
        : points_resource_(nullptr)
        , d_depths_(nullptr)
        , d_depths_out_(nullptr)
        , d_indices_(nullptr)
        , d_indices_out_(nullptr)
        , d_points_temp_(nullptr)
        , d_cub_temp_storage_(nullptr)
        , cub_temp_storage_bytes_(0)
        , max_points_(0)
        , registered_(false)
    {}

    /**
     * Destructor - ensures cleanup
     */
    ~CUDAPointSorter() {
        if (registered_) {
            try {
                unregister();
            } catch (...) {
                // Suppress exceptions in destructor
            }
        }
    }

    /**
     * Register OpenGL SSBO with CUDA
     * 
     * @param points_ssbo_id OpenGL buffer ID for points (PointData array)
     * @param max_points Maximum number of points (for allocating temp buffers)
     */
    void register_buffers(unsigned int points_ssbo_id, size_t max_points) {
        if (registered_) {
            throw std::runtime_error("Buffers already registered. Call unregister() first.");
        }

        max_points_ = max_points;
        
        // Register points SSBO (need both read and write access)
        CUDA_CHECK(cudaGraphicsGLRegisterBuffer(
            &points_resource_,
            points_ssbo_id,
            cudaGraphicsRegisterFlagsNone  // Read/Write access
        ));
        
        // Allocate temporary buffers
        CUDA_CHECK(cudaMalloc(&d_depths_, max_points * sizeof(float)));
        CUDA_CHECK(cudaMalloc(&d_depths_out_, max_points * sizeof(float)));
        CUDA_CHECK(cudaMalloc(&d_indices_, max_points * sizeof(unsigned int)));
        CUDA_CHECK(cudaMalloc(&d_indices_out_, max_points * sizeof(unsigned int)));
        CUDA_CHECK(cudaMalloc(&d_points_temp_, max_points * sizeof(PointData)));
        
        // Determine CUB temporary storage requirements for SortPairs
        cub_temp_storage_bytes_ = 0;
        cub::DeviceRadixSort::SortPairs(
            nullptr, cub_temp_storage_bytes_,
            d_depths_, d_depths_out_,      // keys (separate input/output)
            d_indices_, d_indices_out_,    // values (separate input/output)
            max_points
        );
        
        // Allocate CUB temporary storage
        CUDA_CHECK(cudaMalloc(&d_cub_temp_storage_, cub_temp_storage_bytes_));
        
        registered_ = true;
    }

    /**
     * Sort points by depth and reorder point data
     * 
     * @param depth_coeffs 3-element numpy array [a, b, c] for depth = ax + by + cz
     * @param num_points Number of points to sort
     */
    void sort_by_depth(py::array_t<float> depth_coeffs, size_t num_points) {
        if (!registered_) {
            throw std::runtime_error("Buffers not registered. Call register_buffers() first.");
        }
        
        if (num_points > max_points_) {
            throw std::runtime_error("num_points exceeds max_points");
        }

        // Check depth_coeffs shape
        auto buf = depth_coeffs.request();
        if (buf.size != 3) {
            throw std::runtime_error("depth_coeffs must have 3 elements");
        }
        
        float* h_depth_coeffs = static_cast<float*>(buf.ptr);
        
        // Map OpenGL buffer to CUDA
        CUDA_CHECK(cudaGraphicsMapResources(1, &points_resource_, 0));
        
        try {
            // Get device pointer
            PointData* d_points = nullptr;
            size_t points_size = 0;
            
            CUDA_CHECK(cudaGraphicsResourceGetMappedPointer(
                (void**)&d_points, &points_size, points_resource_));
            
            // Compute depths (coefficients passed directly as kernel parameters)
            const int block_size = 256;
            const int num_blocks = (num_points + block_size - 1) / block_size;
            
            compute_depths_kernel<<<num_blocks, block_size>>>(
                d_points,
                h_depth_coeffs[0], h_depth_coeffs[1], h_depth_coeffs[2],  // Pass as immediate values
                d_depths_,
                d_indices_,
                num_points
            );
            CUDA_CHECK(cudaGetLastError());
            CUDA_CHECK(cudaDeviceSynchronize());
            
            // Sort by depth using CUB radix sort (2-3x faster than Thrust)
            // CUB SortPairs: sorts (depth, index) pairs by depth
            // Requires separate input/output buffers
            cub::DeviceRadixSort::SortPairs(
                d_cub_temp_storage_,
                cub_temp_storage_bytes_,
                d_depths_, d_depths_out_,      // keys (input -> output)
                d_indices_, d_indices_out_,    // values (input -> output)
                num_points
            );
            CUDA_CHECK(cudaGetLastError());
            CUDA_CHECK(cudaDeviceSynchronize());
            
            // Reorder points using custom gather kernel
            const int gather_block_size = 256;
            const int gather_num_blocks = (num_points + gather_block_size - 1) / gather_block_size;
            
            gather_points_kernel<<<gather_num_blocks, gather_block_size>>>(
                d_indices_out_,  // Use output indices from CUB sort
                d_points,
                d_points_temp_,
                num_points
            );
            CUDA_CHECK(cudaGetLastError());
            CUDA_CHECK(cudaDeviceSynchronize());
            
            // Copy reordered data back to original buffer
            CUDA_CHECK(cudaMemcpy(d_points, d_points_temp_, 
                                  num_points * sizeof(PointData), 
                                  cudaMemcpyDeviceToDevice));
            
        } catch (...) {
            // Ensure we unmap resources even on error
            cudaGraphicsUnmapResources(1, &points_resource_, 0);
            throw;
        }
        
        // Unmap resource
        CUDA_CHECK(cudaGraphicsUnmapResources(1, &points_resource_, 0));
    }

    /**
     * Unregister and cleanup CUDA resources
     */
    void unregister() {
        if (!registered_) {
            return;
        }
        
        if (d_depths_) {
            cudaFree(d_depths_);
            d_depths_ = nullptr;
        }
        
        if (d_depths_out_) {
            cudaFree(d_depths_out_);
            d_depths_out_ = nullptr;
        }
        
        if (d_indices_) {
            cudaFree(d_indices_);
            d_indices_ = nullptr;
        }
        
        if (d_indices_out_) {
            cudaFree(d_indices_out_);
            d_indices_out_ = nullptr;
        }
        
        if (d_points_temp_) {
            cudaFree(d_points_temp_);
            d_points_temp_ = nullptr;
        }
        
        if (d_cub_temp_storage_) {
            cudaFree(d_cub_temp_storage_);
            d_cub_temp_storage_ = nullptr;
        }
        
        if (points_resource_) {
            cudaGraphicsUnregisterResource(points_resource_);
            points_resource_ = nullptr;
        }
        
        registered_ = false;
    }

    /**
     * Check if buffers are registered
     */
    bool is_registered() const {
        return registered_;
    }
};


//=============================================================================
// Pybind11 Module Definition
//=============================================================================

PYBIND11_MODULE(cuda_point_sort, m) {
    m.doc() = "CUDA-accelerated point cloud depth sorting with OpenGL interop";

    py::class_<CUDAPointSorter>(m, "CUDAPointSorter")
        .def(py::init<>(), "Create a new CUDA point sorter")
        .def("register_buffers", &CUDAPointSorter::register_buffers,
             py::arg("points_ssbo_id"), py::arg("max_points"),
             "Register OpenGL SSBO with CUDA for interop")
        .def("sort_by_depth", &CUDAPointSorter::sort_by_depth,
             py::arg("depth_coeffs"), py::arg("num_points"),
             "Sort points by depth (far to near) and reorder in-place")
        .def("unregister", &CUDAPointSorter::unregister,
             "Unregister CUDA resources")
        .def("is_registered", &CUDAPointSorter::is_registered,
             "Check if buffers are registered");

    // Module-level functions
    m.def("is_cuda_available", []() {
        int device_count = 0;
        cudaError_t err = cudaGetDeviceCount(&device_count);
        return (err == cudaSuccess && device_count > 0);
    }, "Check if CUDA is available on this system");
}
