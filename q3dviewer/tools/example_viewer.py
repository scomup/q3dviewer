#!/usr/bin/env python3

import q3dviewer as q3d  # Import q3dviewer

def main():
    # Create a Qt application
    app = q3d.QApplication([])

    # Create various 3D items
    axis_item = q3d.AxisItem(size=0.5, width=5)
    grid_item = q3d.GridItem(size=10, spacing=1)

    # Create a viewer
    viewer = q3d.Viewer(name='example')
    
    # Add items to the viewer
    viewer.add_items({
        'grid': grid_item,
        'axis': axis_item,
    })

    # Show the viewer & run the Qt application
    viewer.show()
    app.exec()

if __name__ == '__main__':
    main()
