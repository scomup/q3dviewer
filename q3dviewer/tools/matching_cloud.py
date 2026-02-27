#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.

Match two PCD files using Open3D GICP (Generalized Iterative Closest Point)
with interactive transformation adjustment
"""

import numpy as np
import argparse
import q3dviewer as q3d
from q3dviewer.Qt.QtWidgets import QLabel, QDoubleSpinBox, QVBoxLayout, QPushButton, QFileDialog
from q3dviewer.Qt import QtCore
from q3dviewer.utils.maths import matrix_to_quaternion, euler_to_matrix, matrix_to_euler
from q3dviewer.utils.cloud_io import load_pcd, save_pcd
from q3dviewer.utils.cloud_registration import matching

try:
    import open3d as o3d
except ImportError:
    print(
        "\033[91mWarning: open3d is not installed. Please install it to use this tool.\033[0m")
    print("\033[93mYou can install it using: pip install open3d\033[0m")
    exit(1)


class CustomDoubleSpinBox(QDoubleSpinBox):
    def __init__(self, decimals=4, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.decimals = decimals
        self.setDecimals(self.decimals)

    def textFromValue(self, value):
        return f"{value:.{self.decimals}f}"

    def valueFromText(self, text):
        return float(text)


class GICPMatchingViewer(q3d.Viewer):
    def __init__(self, source_cloud, target_cloud, source_cloud_full=None, voxel_size=0.05, **kwargs):
        self.source_cloud = source_cloud  # Downsampled for display and ICP
        self.target_cloud = target_cloud  # For display and ICP
        self.source_cloud_full = source_cloud_full  # Full resolution for saving
        self.voxel_size = voxel_size
        self.radius = 0.2
        self.t_init = np.array([0, 0, 0])
        self.R_init = np.eye(3)
        super().__init__(**kwargs)

    def default_gl_setting(self, glwidget):
        glwidget.set_bg_color('#ffffff')
        glwidget.set_cam_position(distance=10)

    def add_control_panel(self, main_layout):
        setting_layout = QVBoxLayout()
        setting_layout.setAlignment(QtCore.Qt.AlignTop)

        # XYZ translation
        label_xyz = QLabel("Set XYZ Translation:")
        setting_layout.addWidget(label_xyz)
        self.box_x = CustomDoubleSpinBox()
        self.box_x.setSingleStep(0.01)
        self.box_x.setRange(-100.0, 100.0)
        setting_layout.addWidget(self.box_x)
        self.box_y = CustomDoubleSpinBox()
        self.box_y.setSingleStep(0.01)
        self.box_y.setRange(-100.0, 100.0)
        setting_layout.addWidget(self.box_y)
        self.box_z = CustomDoubleSpinBox()
        self.box_z.setSingleStep(0.01)
        self.box_z.setRange(-100.0, 100.0)
        setting_layout.addWidget(self.box_z)

        # Roll-Pitch-Yaw rotation
        label_rpy = QLabel("Set Roll-Pitch-Yaw:")
        setting_layout.addWidget(label_rpy)
        self.box_roll = CustomDoubleSpinBox()
        self.box_roll.setSingleStep(0.01)
        self.box_roll.setRange(-np.pi, np.pi)
        setting_layout.addWidget(self.box_roll)
        self.box_pitch = CustomDoubleSpinBox()
        self.box_pitch.setSingleStep(0.01)
        self.box_pitch.setRange(-np.pi, np.pi)
        setting_layout.addWidget(self.box_pitch)
        self.box_yaw = CustomDoubleSpinBox()
        self.box_yaw.setSingleStep(0.01)
        self.box_yaw.setRange(-np.pi, np.pi)
        setting_layout.addWidget(self.box_yaw)

        # Radius control
        label_radius = QLabel("Set ICP matching radius:")
        setting_layout.addWidget(label_radius)
        self.box_radius = CustomDoubleSpinBox(decimals=2)
        self.box_radius.setSingleStep(0.1)
        self.box_radius.setRange(0.1, 3.0)
        self.box_radius.setValue(self.radius)
        self.box_radius.valueChanged.connect(self.update_radius)
        setting_layout.addWidget(self.box_radius)

        # ICP matching button
        self.gicp_button = QPushButton("Run ICP Matching")
        self.gicp_button.clicked.connect(self.perform_matching)
        setting_layout.addWidget(self.gicp_button)

        # Save button
        self.save_button = QPushButton("Save Aligned Cloud")
        self.save_button.clicked.connect(self.save_aligned)
        setting_layout.addWidget(self.save_button)

        # Connect value changes
        self.box_x.valueChanged.connect(self.apply_transform)
        self.box_y.valueChanged.connect(self.apply_transform)
        self.box_z.valueChanged.connect(self.apply_transform)
        self.box_roll.valueChanged.connect(self.apply_transform)
        self.box_pitch.valueChanged.connect(self.apply_transform)
        self.box_yaw.valueChanged.connect(self.apply_transform)

        main_layout.addLayout(setting_layout)

    def update_radius(self):
        self.radius = self.box_radius.value()

    def apply_transform(self):
        """Update transformation parameters and apply to source using set_transform."""
        self.t_init = np.array([
            self.box_x.value(),
            self.box_y.value(),
            self.box_z.value()
        ])
        rpy = np.array([
            self.box_roll.value(),
            self.box_pitch.value(),
            self.box_yaw.value()
        ])
        self.R_init = euler_to_matrix(rpy)

        # Apply transformation to source
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = self.R_init
        T[:3, 3] = self.t_init
        self['source'].set_transform(T)

    def perform_matching(self):
        print("\nPerforming scan matching registration...")

        # Initial transformation
        T_init = np.eye(4)
        T_init[:3, :3] = self.R_init
        T_init[:3, 3] = self.t_init

        # Perform ICP registration using cloud_registration.matching
        T_result, result = matching(
            self.target_cloud, self.source_cloud,
            down_sampling_size=self.voxel_size,
            radius=self.radius,
            T_init=T_init,
            icp_model="gicp"
        )

        # Update transformation from result
        self.R_init = T_result[:3, :3]
        self.t_init = T_result[:3, 3]

        # Update UI (block signals to prevent multiple intermediate updates)
        rpy = matrix_to_euler(self.R_init)
        self.box_roll.blockSignals(True)
        self.box_pitch.blockSignals(True)
        self.box_yaw.blockSignals(True)
        self.box_x.blockSignals(True)
        self.box_y.blockSignals(True)
        self.box_z.blockSignals(True)

        self.box_roll.setValue(rpy[0])
        self.box_pitch.setValue(rpy[1])
        self.box_yaw.setValue(rpy[2])
        self.box_x.setValue(self.t_init[0])
        self.box_y.setValue(self.t_init[1])
        self.box_z.setValue(self.t_init[2])

        self.box_roll.blockSignals(False)
        self.box_pitch.blockSignals(False)
        self.box_yaw.blockSignals(False)
        self.box_x.blockSignals(False)
        self.box_y.blockSignals(False)
        self.box_z.blockSignals(False)

        print("\nICP Registration Result:")
        print(f"Fitness: {result.fitness:.6f}")

        # Update visualization once with final result
        self.apply_transform()

    def save_aligned(self):
        """Save the aligned source cloud to a PCD file using full resolution data."""
        # Create transformation matrix
        T = np.eye(4)
        T[:3, :3] = self.R_init
        T[:3, 3] = self.t_init

        # Use full resolution if available, otherwise use downsampled
        source_to_save = self.source_cloud_full if self.source_cloud_full is not None else self.source_cloud

        # Transform point cloud
        source_transformed = source_to_save.copy()
        xyz = source_transformed['xyz']
        xyz_homogeneous = np.column_stack([xyz, np.ones(len(xyz))])
        xyz_transformed = (T @ xyz_homogeneous.T).T[:, :3]
        source_transformed['xyz'] = xyz_transformed

        # Open file dialog to select save location
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save Aligned Point Cloud",
            "aligned_source.pcd",
            "PCD Files (*.pcd);;All Files (*)"
        )

        if filename:
            save_pcd(source_transformed, filename)
            print(f"Saved aligned source cloud to: {filename}")
            print(f"Full resolution points saved: {len(source_transformed)}")
        else:
            print("Save cancelled")


def numpy_to_o3d(cloud_np):
    """Convert numpy structured array to Open3D point cloud."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud_np['xyz'])
    return pcd


def main():
    parser = argparse.ArgumentParser(
        description="Find a T let T * source = target using ICP registration with interactive adjustment")
    parser.add_argument("source", type=str, help="Path to source PCD file")
    parser.add_argument("target", type=str, help="Path to target PCD file")
    parser.add_argument("--voxel_size", type=float, default=0.05,
                        help="Voxel size for downsampling (default: 0.05)")
    parser.add_argument("--max_points", type=int, default=10000000,
                        help="Maximum points to visualize (default: 10000000)")
    args = parser.parse_args()

    # Load point clouds
    print(f"Loading source: {args.source}")
    source_np_full = load_pcd(args.source)
    print(f"Source points: {len(source_np_full)}")

    print(f"Loading target: {args.target}")
    target_np = load_pcd(args.target)
    print(f"Target points: {len(target_np)}")

    # Downsample source for visualization
    source_np = source_np_full
    if source_np_full.size > args.max_points:
        print(
            f"Source cloud has more than {args.max_points} points, randomly sampling for visualization...")
        source_np = source_np_full[np.random.choice(
            len(source_np_full), size=args.max_points, replace=False)]

    # Downsample target for visualization
    if target_np.size > args.max_points:
        print(
            f"Target cloud has more than {args.max_points} points, randomly sampling for visualization...")
        target_np = target_np[np.random.choice(
            len(target_np), size=args.max_points, replace=False)]

    # Create interactive viewer
    app = q3d.QApplication(["Matching Cloud"])
    viewer = GICPMatchingViewer(source_np, target_np, source_np_full, args.voxel_size,
                                name='Matching Cloud')

    # Add visualization items
    grid_item = q3d.GridItem(size=100, spacing=10, color='#00000040')
    source_item = q3d.CloudItem(
        size=1, alpha=1, color_mode='FLAT', color='#ff0000')
    target_item = q3d.CloudItem(
        size=1, alpha=1, color_mode='FLAT', color='#00ff00')

    viewer.add_items({
        'grid': grid_item,
        'source': source_item,
        'target': target_item
    })

    # Set initial data
    viewer['source'].set_data(source_np)
    viewer['target'].set_data(target_np)
    viewer.apply_transform()

    viewer.show()
    app.exec()


if __name__ == "__main__":
    main()
