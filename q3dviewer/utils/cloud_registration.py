#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.

Point cloud registration utilities using Open3D ICP algorithms.
"""

import numpy as np

try:
    import open3d as o3d
except ImportError:
    print("\033[91mWarning: open3d is not installed. Please install it to use this module.\033[0m")
    print("\033[93mYou can install it using: pip install open3d\033[0m")
    o3d = None


def matching(cloud0, cloud1, down_sampling_size=0.1, radius=0.2, T_init=None, icp_model="gicp"):
    """
    Perform ICP registration between two point clouds.
    
    Find transformation T such that T * cloud1 ≈ cloud0
    
    Args:
        cloud0: Target point cloud (numpy structured array with 'xyz' field)
        cloud1: Source point cloud to be transformed (numpy structured array with 'xyz' field)
        down_sampling_size: Voxel size for downsampling (default: 0.1)
        radius: Search radius for normal estimation and ICP matching (default: 0.2)
        T_init: Initial transformation matrix (4x4 numpy array), defaults to identity
        icp_model: ICP algorithm to use, either "gicp" or "p2plane" (default: "gicp")
    
    Returns:
        T: Transformation matrix (4x4 numpy array) such that T * cloud1 ≈ cloud0
        result: Open3D registration result object containing fitness and other metrics
    
    Example:
        >>> T, result = matching(target_cloud, source_cloud, down_sampling_size=0.05)
        >>> print(f"Fitness: {result.fitness:.6f}")
        >>> print(f"Transformation:\\n{T}")
    """
    if o3d is None:
        raise ImportError("open3d is required for point cloud registration")
    
    # Convert to Open3D format
    cloud0_o3d = o3d.geometry.PointCloud()
    cloud0_o3d.points = o3d.utility.Vector3dVector(cloud0['xyz'])
    cloud1_o3d = o3d.geometry.PointCloud()
    cloud1_o3d.points = o3d.utility.Vector3dVector(cloud1['xyz'])
    
    # Downsample
    cloud0_o3d = cloud0_o3d.voxel_down_sample(down_sampling_size)
    cloud1_o3d = cloud1_o3d.voxel_down_sample(down_sampling_size)
    
    # Estimate normals
    cloud0_o3d.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius, max_nn=30))
    cloud1_o3d.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius, max_nn=30))
    
    # Initial transformation
    if T_init is None:
        T_init = np.eye(4)
    
    # Define convergence criteria
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
        relative_fitness=1e-4,
        max_iteration=50)
    
    # Perform registration based on model type
    if icp_model == "p2plane":
        # Point-to-plane ICP
        result = o3d.pipelines.registration.registration_icp(
            cloud1_o3d, cloud0_o3d, radius, T_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria)
    elif icp_model == "gicp":
        # Check Open3D version for GICP support
        if o3d.__version__ < "0.15.0":
            print("\033[93mWarning: Open3D version is older than 0.15.0. Falling back to point-to-plane ICP.\033[0m")
            result = o3d.pipelines.registration.registration_icp(
                cloud1_o3d, cloud0_o3d, radius, T_init,
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                criteria)
        else:
            # GICP - need Covariances
            cloud0_o3d.estimate_covariances(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=radius, max_nn=30))
            cloud1_o3d.estimate_covariances(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=radius, max_nn=30))
            result = o3d.pipelines.registration.registration_generalized_icp(
                cloud1_o3d, cloud0_o3d, radius, T_init,
                o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
                criteria)
    else:
        raise ValueError(f"Unsupported ICP model: {icp_model}. Use 'gicp' or 'p2plane'.")
    
    return result.transformation, result
