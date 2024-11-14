#!/usr/bin/env python3


from q3dviewer import *
import rospy

class ViewerWithPanel(Viewer):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
    def initUI(self):
        centerWidget = QWidget()
        self.setCentralWidget(centerWidget)
        main_layout = QHBoxLayout()
        centerWidget.setLayout(main_layout)

        # Create a vertical layout for the buttons
        button_layout = QVBoxLayout()
        button1 = QPushButton("Button 1")
        button2 = QPushButton("Button 2")
        button3 = QPushButton("Button 3")
        button_layout.addWidget(button1)
        button_layout.addWidget(button2)
        button_layout.addWidget(button3)
        button_layout.addStretch(1)  # Add a stretch to push the buttons to the top
    
        self.viewerWidget = self.vw()
        main_layout.addLayout(button_layout)
        main_layout.addWidget(self.viewerWidget, 1)
        timer = QtCore.QTimer(self)
        timer.setInterval(20)  # period, in milliseconds
        timer.timeout.connect(self.update)
        self.viewerWidget.setCameraPosition(distance=40)
        timer.start()


def main():
    rospy.init_node('lidar_cam_manual_calib', anonymous=True)
    app = QApplication([])
    gird_item = GridItem(size=10, spacing=1)
    viewer = ViewerWithPanel(name='example')
    viewer.addItems({'grid': gird_item})

    viewer.show()
    app.exec_()

if __name__ == '__main__':
    main()



