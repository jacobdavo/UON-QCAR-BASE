#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Float64

import sys
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication, QLabel, QGridLayout, QWidget, QSlider


class ControlNode(object):
    def __init__(self):
        super().__init__()
        self.publishers()
        self.now = rospy.Time.now()

    def publishers(self):
        self.rl_wheel_pub = rospy.Publisher('/wheelrl_motor/command', Float32, queue_size=0)
        self.rr_wheel_pub = rospy.Publisher('/wheelrr_motor/command', Float32, queue_size=0)
        self.fl_wheel_pub = rospy.Publisher('/wheelfl_motor/command', Float32, queue_size=0)
        self.fr_wheel_pub = rospy.Publisher('/wheelfr_motor/command', Float32, queue_size=0)

        self.fl_steering_wheel_pub = rospy.Publisher('/qcar/base_fl_controller/command', Float64, queue_size=0)
        self.fr_steering_wheel_pub = rospy.Publisher('/qcar/base_fr_controller/command', Float64, queue_size=0)

    def velocity_command(self, value):
        self.rr_wheel_pub.publish(value)
        self.rl_wheel_pub.publish(Float32(-value))
        self.fr_wheel_pub.publish(value)
        self.fl_wheel_pub.publish(Float32(-value))

    def steering_command(self, value):
        self.fl_steering_wheel_pub.publish(value)
        self.fr_steering_wheel_pub.publish(value)


class CONTROLWindow(QWidget):
    def __init__(self, node):
        super(CONTROLWindow, self).__init__()

        master_layout = QGridLayout(self)
        self.setWindowTitle('UON AUTONOMOUS TEST TOOL')

        self.velocity_value = 0
        self.steering_value = 0

        self.velocity_slider = QSlider(Qt.Vertical, self)
        self.velocity_slider.setTickPosition(QSlider.TicksBothSides)
        self.velocity_slider.setTickInterval(1)
        self.velocity_slider.setSingleStep(1)
        self.velocity_slider.setMaximum(10)
        self.velocity_slider.setMinimum(-10)
        self.velocity_slider.setValue(0)
        self.velocity_slider.valueChanged.connect(self.updateVelocity)
        font = self.font()
        font.setPointSize(24)
        self.velocity_label = QLabel("{}".format(self.velocity_value), self)
        self.steering_label = QLabel("{}".format(self.steering_value), self)
        self.velocity_output = QLabel('km/h', self)
        self.steering_output = QLabel('degs', self)

        self.steering_slider = QSlider(Qt.Horizontal)
        self.steering_slider.setTickPosition(QSlider.TicksBothSides)
        self.steering_slider.setTickInterval(1)
        self.steering_slider.setSingleStep(1)
        self.steering_slider.setMaximum(30)
        self.steering_slider.setMinimum(-30)
        self.steering_slider.setValue(0)

        self.steering_slider.valueChanged.connect(self.updateSteering)

        master_layout.addWidget(self.velocity_slider,   0,0, alignment=Qt.AlignHCenter)
        master_layout.addWidget(self.velocity_label,    0,1)
        master_layout.addWidget(self.velocity_output,  0,2)
        master_layout.addWidget(self.steering_slider,   1,0)
        master_layout.addWidget(self.steering_label,    1,1)
        master_layout.addWidget(self.steering_output,  1,2)
        self.setLayout(master_layout)
        self.resize(600, 600)

        self.send_topic_timer = QTimer(self)
        self.send_topic_timer.timeout.connect(self.set_parameters)
        self.send_topic_timer.start(100)

        self.show()

    def updateVelocity(self, value):
        self.velocity_value = (value/0.0325)/3.6
        self.velocity_label.setText(str(value))

    def updateSteering(self, value):
        self.steering_value = value*0.01745329251
        self.steering_label.setText(str(value))

    def set_parameters(self):
        ControlNode().velocity_command(self.velocity_value)
        ControlNode().steering_command(self.steering_value)
        

if __name__ == '__main__':
    rospy.init_node('control_node')
    r = ControlNode()
    app = QApplication(sys.argv)
    window = CONTROLWindow(0)
    sys.exit(app.exec_())
    rospy.spin()