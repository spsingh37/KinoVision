#!/usr/bin/python
"""!
Main GUI for Arm lab
"""
import os, sys
script_path = os.path.dirname(os.path.realpath(__file__))

import argparse
import cv2
import numpy as np
import rclpy
import time
from functools import partial

from PyQt5.QtCore import QThread, Qt, pyqtSignal, pyqtSlot, QTimer
from PyQt5.QtGui import QPixmap, QImage, QCursor
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QMainWindow, QFileDialog

from resource.ui import Ui_MainWindow
from rxarm import RXArm, RXArmThread
from camera import Camera, VideoThread
from state_machine import StateMachine, StateMachineThread
""" Radians to/from  Degrees conversions """
D2R = np.pi / 180.0
R2D = 180.0 / np.pi


class Gui(QMainWindow):
    """!
    Main GUI Class

    Contains the main function and interfaces between the GUI and functions.
    """
    # def __init__(self, parent=None, dh_config_file=None):
    # add parameter to read pox_config_file
    def __init__(self, parent=None, dh_config_file=None, pox_config_file=None):
        QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        """ Groups of ui commonents """
        self.joint_readouts = [
            self.ui.rdoutBaseJC,
            self.ui.rdoutShoulderJC,
            self.ui.rdoutElbowJC,
            self.ui.rdoutWristAJC,
            self.ui.rdoutWristRJC,
        ]
        self.joint_slider_rdouts = [
            self.ui.rdoutBase,
            self.ui.rdoutShoulder,
            self.ui.rdoutElbow,
            self.ui.rdoutWristA,
            self.ui.rdoutWristR,
        ]
        self.joint_sliders = [
            self.ui.sldrBase,
            self.ui.sldrShoulder,
            self.ui.sldrElbow,
            self.ui.sldrWristA,
            self.ui.sldrWristR,
        ]
        """Objects Using Other Classes"""
        self.camera = Camera()
        print("Creating rx arm...")
        if (dh_config_file is not None):
            self.rxarm = RXArm(dh_config_file=dh_config_file)
        else:
            # self.rxarm = RXArm()
            # read pox.csv file
            self.rxarm = RXArm(pox_config_file=pox_config_file)
        print("Done creating rx arm instance.")
        self.sm = StateMachine(self.rxarm, self.camera)
        """
        Attach Functions to Buttons & Sliders
        TODO: NAME AND CONNECT BUTTONS AS NEEDED
        """
        # Video
        self.ui.videoDisplay.setMouseTracking(True)
        self.ui.videoDisplay.mouseMoveEvent = self.trackMouse
        self.ui.videoDisplay.mousePressEvent = self.calibrateMousePress

        # Buttons
        # Handy lambda function falsethat can be used with Partial to only set the new state if the rxarm is initialized
        #nxt_if_arm_init = lambda next_state: self.sm.set_next_state(next_state if self.rxarm.initialized else None)
        nxt_if_arm_init = lambda next_state: self.sm.set_next_state(next_state)
        self.ui.btn_estop.clicked.connect(self.estop)
        self.ui.btn_init_arm.clicked.connect(self.initRxarm)
        self.ui.btn_torq_off.clicked.connect(
            lambda: self.rxarm.disable_torque())
        self.ui.btn_torq_on.clicked.connect(lambda: self.rxarm.enable_torque())
        self.ui.btn_sleep_arm.clicked.connect(lambda: self.rxarm.sleep())
        self.ui.btn_task1.clicked.connect(partial(nxt_if_arm_init, 'event1'))
        self.ui.btn_task2.clicked.connect(partial(nxt_if_arm_init, 'event2'))
        self.ui.btn_task3.clicked.connect(partial(nxt_if_arm_init, 'event3'))
        self.ui.btn_task4.clicked.connect(partial(nxt_if_arm_init, 'event4'))
        self.ui.btn_task5.clicked.connect(partial(nxt_if_arm_init, 'bonus'))



        #User Buttons
        self.ui.btnUser1.setText("Calibrate")
        self.ui.btnUser1.clicked.connect(partial(nxt_if_arm_init, 'calibrate'))
        self.ui.btnUser2.setText('Open Gripper')
        self.ui.btnUser2.clicked.connect(lambda: self.gripper_open())
        self.ui.btnUser3.setText('Close Gripper')
        self.ui.btnUser3.clicked.connect(lambda: self.gripper_close())
        self.ui.btnUser4.setText('Execute')
        self.ui.btnUser4.clicked.connect(partial(nxt_if_arm_init, 'execute'))
        self.ui.btnUser5.setText('Teach')
        self.ui.btnUser5.clicked.connect(partial(nxt_if_arm_init, 'Teach'))
        self.ui.btnUser6.setText('Repeat')
        self.ui.btnUser6.clicked.connect(partial(nxt_if_arm_init, 'Repeat'))
        self.ui.btnUser7.setText('Clear')
        self.ui.btnUser7.clicked.connect(partial(nxt_if_arm_init, 'Clear'))
        self.ui.btnUser8.setText('Click_to_grab')
        self.ui.btnUser8.clicked.connect(partial(nxt_if_arm_init, 'click_to_grab'))
        self.ui.btnUser9.setText('Click_to_drop')
        self.ui.btnUser9.clicked.connect(partial(nxt_if_arm_init, 'click_to_drop'))
        self.ui.btnUser10.setText('Compare')
        self.ui.btnUser10.clicked.connect(partial(nxt_if_arm_init, 'compare'))
        self.ui.btnUser11.setText('Detect')
        self.ui.btnUser11.clicked.connect(partial(nxt_if_arm_init, 'detect'))


        # Sliders
        for sldr in self.joint_sliders:
            sldr.valueChanged.connect(self.sliderChange)
        self.ui.sldrMoveTime.valueChanged.connect(self.sliderChange)
        self.ui.sldrAccelTime.valueChanged.connect(self.sliderChange)
        # Direct Control
        self.ui.chk_directcontrol.stateChanged.connect(self.directControlChk)
        # Status
        self.ui.rdoutStatus.setText("Waiting for input")
        """initalize manual control off"""
        self.ui.SliderFrame.setEnabled(False)
        """Setup Threads"""

        # State machine
        self.StateMachineThread = StateMachineThread(self.sm)
        self.StateMachineThread.updateStatusMessage.connect(
            self.updateStatusMessage)
        self.StateMachineThread.start()
        self.VideoThread = VideoThread(self.camera)
        self.VideoThread.updateFrame.connect(self.setImage)
        self.VideoThread.start()
        self.ArmThread = RXArmThread(self.rxarm)
        self.ArmThread.updateJointReadout.connect(self.updateJointReadout)
        self.ArmThread.updateEndEffectorReadout.connect(
            self.updateEndEffectorReadout)
        self.ArmThread.start()

    """ Slots attach callback functions to signals emitted from threads"""

    @pyqtSlot(str)
    def updateStatusMessage(self, msg):
        self.ui.rdoutStatus.setText(msg)

    @pyqtSlot(list)
    def updateJointReadout(self, joints):
        for rdout, joint in zip(self.joint_readouts, joints):
            rdout.setText(str('%+.2f' % (joint * R2D)))

    ### TODO: output the rest of the orientation according to the convention chosen
    @pyqtSlot(list)
    def updateEndEffectorReadout(self, pos):
        self.ui.rdoutX.setText(str("%+.2f mm" % (1000 * pos[0])))
        self.ui.rdoutY.setText(str("%+.2f mm" % (1000 * pos[1])))
        self.ui.rdoutZ.setText(str("%+.2f mm" % (1000 * pos[2])))
        self.ui.rdoutPhi.setText(str("%+.2f rad" % (pos[3])))
        self.ui.rdoutTheta.setText(str("%+.2f" % (pos[4])))
        self.ui.rdoutPsi.setText(str("%+.2f" % (pos[5])))

    @pyqtSlot(QImage, QImage, QImage, QImage)
    def setImage(self, rgb_image, depth_image, tag_image, grid_image):
        """!
        @brief      Display the images from the camera.

        @param      rgb_image    The rgb image
        @param      depth_image  The depth image
        """
        if (self.ui.radioVideo.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(rgb_image))
        if (self.ui.radioDepth.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(depth_image))
        if (self.ui.radioUsr1.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(tag_image))
        if (self.ui.radioUsr2.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(grid_image))
        # if (self.ui.radioUsr3.isChecked()):
        #     self.ui.videoDisplay.setPixmap(QPixmap.fromImage(block_frame))

    """ Other callback functions attached to GUI elements"""

    def estop(self):
        self.rxarm.disable_torque()
        self.sm.set_next_state('estop')

    def gripper_open(self):
        self.rxarm.gripper.release()
        self.rxarm.gripper_state = True
    
    def gripper_close(self):
        self.rxarm.gripper.grasp()
        self.rxarm.gripper_state = False


    def sliderChange(self):
        """!
        @brief Slider changed

        Function to change the slider labels when sliders are moved and to command the arm to the given position
        """
        for rdout, sldr in zip(self.joint_slider_rdouts, self.joint_sliders):
            rdout.setText(str(sldr.value()))

        self.ui.rdoutMoveTime.setText(
            str(self.ui.sldrMoveTime.value() / 10.0) + "s")
        self.ui.rdoutAccelTime.setText(
            str(self.ui.sldrAccelTime.value() / 20.0) + "s")
        self.rxarm.set_moving_time(self.ui.sldrMoveTime.value() / 10.0)
        self.rxarm.set_accel_time(self.ui.sldrAccelTime.value() / 20.0)

        # Do nothing if the rxarm is not initialized
        if self.rxarm.initialized:
            joint_positions = np.array(
                [sldr.value() * D2R for sldr in self.joint_sliders])
            # Only send the joints that the rxarm has
            self.rxarm.set_positions(joint_positions[0:self.rxarm.num_joints])

    def directControlChk(self, state):
        """!
        @brief      Changes to direct control mode

                    Will only work if the rxarm is initialized.

        @param      state  State of the checkbox
        """
        if state == Qt.Checked and self.rxarm.initialized:
            # Go to manual and enable sliders
            self.sm.set_next_state("manual")
            self.ui.SliderFrame.setEnabled(True)
        else:
            # Lock sliders and go to idle
            self.sm.set_next_state("idle")
            self.ui.SliderFrame.setEnabled(False)
            self.ui.chk_directcontrol.setChecked(False)
            
    def world_cood(self,pt):
        old_u = pt.x()
        old_v = pt.y()
        old = np.array([[old_u],[old_v],[1]])
        new2 = np.dot(np.linalg.inv(self.camera.H),old)
        x = int(new2[0][0]/new2[2][0])
        y = int(new2[1][0]/new2[2][0])
        z = int(new2[2]/new2[2])

        x = max(100, min(x, 1170))
        y = max(25, min(y, 700))

        ######task 2.2
        d = self.camera.DepthFrameRaw[y][x]

    
        intrix = self.camera.intrinsic_matrix
        # extrix = self.camera.extrinsic_matrix

        # Task 2.2 automatic calibration
        extrix = self.camera.auto_extrinsic

        proj_mat = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]])
        proj_mat = np.dot(intrix,proj_mat)
        ##image frame to camera frame
        mouse_mat = np.array([[x],[y],[1]])
        intrix_cal = d * np.linalg.inv(intrix)
        cam_coord = np.ones((4,1))
        cam_coord[:3] = np.dot(intrix_cal,mouse_mat)
        
        #camera frame to world frame
        world_coord = np.dot(np.linalg.inv(extrix),cam_coord)
        return world_coord[:3]

    def trackMouse(self, mouse_event):
        """!
        @brief      Show the mouse position in GUI

                    TODO: after implementing workspace calibration display the world coordinates the mouse points to in the RGB
                    video image.

        @param      mouse_event  QtMouseEvent containing the pose of the mouse at the time of the event not current time
        """
        
        pt = mouse_event.pos()
        if self.camera.DepthFrameRaw.any() != 0:
            z = self.camera.DepthFrameRaw[pt.y()][pt.x()]
            world_coordinate = self.world_cood(pt)
            self.ui.rdoutMousePixels.setText("(%.0f,%.0f,%.0f)" %
                                             (pt.x(), pt.y(), z))
            self.ui.rdoutMouseWorld.setText("(%.0f,%.0f,%.0f)" %
                                             (world_coordinate[0], world_coordinate[1], world_coordinate[2]))

    def calibrateMousePress(self, mouse_event):
        """!
        @brief Record mouse click positions for calibration

        @param      mouse_event  Qt
         containing the pose of the mouse at the time of the event not current time
        """
        """ Get mouse posiiton """
        pt = mouse_event.pos()
        self.camera.last_click[0] = pt.x()
        self.camera.last_click[1] = pt.y()
        self.camera.new_click = True
        # print(self.camera.last_click)

    def initRxarm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.ui.SliderFrame.setEnabled(False)
        self.ui.chk_directcontrol.setChecked(False)
        self.rxarm.enable_torque()
        self.sm.set_next_state('initialize_rxarm')


### TODO: Add ability to parse POX config file as well
def main(args=None):
    """!
    @brief      Starts the GUI
    """
    app = QApplication(sys.argv)
    # Task 2.4 Forward Kinematics using DH table
    # app_window = Gui(dh_config_file=args['dhconfig'])

    # Task 2.4 Forward Kinematics using product of exp
    app_window = Gui(pox_config_file=args['poxconfig'])
    app_window.show()

    # Set thread priorities
    app_window.VideoThread.setPriority(QThread.HighPriority)
    app_window.ArmThread.setPriority(QThread.NormalPriority)
    app_window.StateMachineThread.setPriority(QThread.LowPriority)

    sys.exit(app.exec_())


# Run main if this file is being run directly
### TODO: Add ability to parse POX config file as well
if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-c",
                    "--dhconfig",
                    required=False,
                    help="path to DH parameters csv file")
    ap.add_argument("-p", 
                    "--poxconfig", 
                    required=False, 
                    help="/home/student_pm/armlab-f23/config/rx200_pox.csv")
    main(args=vars(ap.parse_args()))
