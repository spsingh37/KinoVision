"""!
The state machine that implements the logic.
"""
from PyQt5.QtCore import QThread, Qt, pyqtSignal, pyqtSlot, QTimer
import time
import numpy as np
import rclpy
import cv2

from kinematics import FK_dh, FK_pox, get_pose_from_T, IK_geometric


class StateMachine():
    """!
    @brief      This class describes a state machine.

                TODO: Add states and state functions to this class to implement all of the required logic for the armlab
    """

    def __init__(self, rxarm, camera):
        """!
        @brief      Constructs a new instance.

        @param      rxarm   The rxarm
        @param      planner  The planner
        @param      camera   The camera
        """
        self.rxarm = rxarm
        self.camera = camera
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.record = []
        self.pre_ee_pose = []
        self.waypoints = [
            [-np.pi/2,       -0.5,      -0.3,          0.0,        0.0],
            [0.75*-np.pi/2,   0.5,       0.3,     -np.pi/3,    np.pi/2],
            [0.5*-np.pi/2,   -0.5,      -0.3,      np.pi/2,        0.0],
            [0.25*-np.pi/2,   0.5,       0.3,     -np.pi/3,    np.pi/2],
            [0.0,             0.0,       0.0,          0.0,        0.0],
            [0.25*np.pi/2,   -0.5,      -0.3,          0.0,    np.pi/2],
            [0.5*np.pi/2,     0.5,       0.3,     -np.pi/3,        0.0],
            [0.75*np.pi/2,   -0.5,      -0.3,          0.0,    np.pi/2],
            [np.pi/2,         0.5,       0.3,     -np.pi/3,        0.0],
            [0.0,             0.0,       0.0,          0.0,        0.0]]

    def set_next_state(self, state):
        """!
        @brief      Sets the next state.

            This is in a different thread than run so we do nothing here and let run handle it on the next iteration.

        @param      state  a string representing the next state.
        """
        self.next_state = state

    def run(self):
        """!
        @brief      Run the logic for the next state

                    This is run in its own thread.

                    TODO: Add states and funcitons as needed.
        """
        if self.next_state == "initialize_rxarm":
            self.initialize_rxarm()

        if self.next_state == "idle":
            self.idle()

        if self.next_state == "estop":
            self.estop()

        if self.next_state == "execute":
            self.execute()

        if self.next_state == "calibrate":
            self.calibrate()

        if self.next_state == "detect":
            self.detect()

        if self.next_state == "manual":
            self.manual()

        if self.next_state == "Teach":
            self.teach()

        if self.next_state == "Repeat":
            self.repeat()

        if self.next_state == "Clear":
            self.Clear()

        if self.next_state == "click_to_grab":
            self.click_to_grab()

        if self.next_state == "click_to_drop":
            self.click_to_drop()

        if self.next_state == "move_to":
            self.move_to()

        if self.next_state == "compare":
            self.compare()

        if self.next_state == "detect":
            self.detect()

        if self.next_state == "event1":
            self.event1()

        if self.next_state == "event2":
            self.event2()

        if self.next_state == "event3":
            self.event3()

        if self.next_state == "event4":
            self.event4()

        if self.next_state == "bonus":
            self.bonus()
    """Functions run for each state"""

    def manual(self):
        """!
        @brief      Manually control the rxarm
        """
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"

    def idle(self):
        """!
        @brief      Do nothing
        """
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"

    def estop(self):
        """!
        @brief      Emergency stop disable torque.
        """
        self.status_message = "EMERGENCY STOP - Check rxarm and restart program"
        self.current_state = "estop"
        self.rxarm.disable_torque()

    def execute(self):
        """!
        @brief      Go through all waypoints
        TODO: Implement this function to execute a waypoint plan
              Make sure you respect estop signal
        """
        self.status_message = "State: Execute - Executing motion plan"
        self.current_state = "execute"
        for position in self.waypoints:
            print(position)
            self.rxarm.set_positions(position)
            time.sleep(3)
            if self.current_state == "estop":
                self.estop()

        self.next_state = "idle"

    def teach(self):
        temp = [self.rxarm.get_positions(),self.rxarm.gripper_state]
        print(self.rxarm.gripper_state)
        self.record.append(temp)
        self.next_state = "idle"
    
    def Clear(self):
        self.record = []
    
    def repeat(self):
        for i in range(10):
            for position in self.record:
                print(position)
                self.rxarm.set_positions(position[0])
                if position[1]==True:
                    self.rxarm.gripper.release()
                else:
                    self.rxarm.gripper.grasp()
                time.sleep(2)
                if self.current_state == "estop":
                    self.estop()

        self.next_state = "idle"

    def click_to_grab(self):
        self.current_state = "click_to_grab"
        self.status_message = "Testing Inverse Kinematics"
        
        mouse_cords = self.camera.last_click
        print("mouse_cords")
        print(mouse_cords)
        world_cords = self.camera.world_cood(mouse_cords[0], mouse_cords[1])
        print("world_cords")
        print(world_cords)
        world_cords = np.array(world_cords) * 0.001
        self.move_to(world_cords[0], world_cords[1], world_cords[2] + 0.02, -np.pi/2)
        time.sleep(2)

        self.move_to(world_cords[0], world_cords[1], world_cords[2] - 0.01, -np.pi/2)
        time.sleep(2)

        self.rxarm.gripper.grasp()
        time.sleep(2)
        
        self.pre_ee_pose = world_cords
        self.next_state = "idle"   
        
    def click_to_drop(self):
        self.current_state = "click_to_drop"
        world_cords = self.pre_ee_pose
        self.move_to(world_cords[0], world_cords[1], world_cords[2] + 0.1, -np.pi/2)
        time.sleep(2)

        mouse_cords = self.camera.last_click
        world_cords = self.camera.world_cood(mouse_cords[0], mouse_cords[1])
        world_cords = np.array(world_cords) * 0.001
        self.move_to(world_cords[0], world_cords[1], world_cords[2] + 0.1, -np.pi/2)
        time.sleep(2)

        self.move_to(world_cords[0], world_cords[1], world_cords[2] + 0.03, -np.pi/2)
        time.sleep(2)

        self.rxarm.gripper.release()
        time.sleep(2)

        self.initialize_rxarm()
        self.pre_ee_pose = []

        self.next_state = "idle"
    
    def move_to(self, x, y, z, psi):
        self.current_state = "move_to"
        world_cords = np.array([x, y, z, psi], dtype=np.float64)
        joint_angles = IK_geometric(world_cords)
        self.rxarm.set_positions(joint_angles)
        time.sleep(1)

        self.next_state = "idle"

    def event1(self):
        self.current_state = "event1"

        small_list = []
        large_list = []
    
        # Level 2: Aggregate positions based on size
        # for color, data in self.camera.detections.items():
        #     positions, angles, sizes, world_pos = data
        #     for i in range(len(sizes)):
        #         if(sizes[i] == "big"):
        #             large_list.append(world_pos[i])
        #         else:
        #             small_list.append(world_pos[i])

        # Level 3: Aggregate positions based on size
        for color, data in self.camera.detections.items():
            positions, angles, sizes, world_pos = data
            print("orig_world_pos", world_pos)
            for i in range(len(sizes)):
                if(sizes[i] == "big" and world_pos[i][2] > 57):
                    large_list.append(world_pos[i])
                    print("ifcond_world_pos",world_pos)
                    world_pos[i][2] = world_pos[i][2] - 30
                    print("new_ifocnd_world_pos",world_pos)
                    large_list.append(world_pos[i])
                    print("1st condition")
                elif (sizes[i] == "small" and world_pos[i][2] > 27 and world_pos[i][2] < 45):
                    small_list.append(world_pos[i])
                    world_pos[i][2] = world_pos[i][2] - 15
                    small_list.append(world_pos[i])
                    print("2nd condition")
                elif (sizes[i] == "small" and world_pos[i][2] > 45 and world_pos[i][2] < 57):
                    small_list.append(world_pos[i])
                    world_pos[i][2] = world_pos[i][2] - 15
                    large_list.append(world_pos[i])
                    print("3rd condition")
                elif(sizes[i] == "big"):
                    large_list.append(world_pos[i])
                    print("4th condition")
                else:
                    small_list.append(world_pos[i])
                    print("5th condition")
        
        large_list = np.array(large_list)*0.001
        small_list = np.array(small_list)*0.001
        print("large_list",large_list)
        print("small_list",small_list)

        small_drop_list = np.array([[-300,-75,0], [-250,-75,0], [-200, -75, 0], [-150, -75, 0], [-120, -60, 0]])*0.001
        large_drop_list = np.array([[200,-80,0], [300,-80, 0], [333, -30, 0], [350, -50, 0], [150, -30, 0]])*0.001

        for i in range(len(small_list)):
            self.move_to(small_list[i][0], small_list[i][1], small_list[i][2] + 0.1, -np.pi/2)
            time.sleep(0.5)
            self.move_to(small_list[i][0], small_list[i][1], small_list[i][2], -np.pi/2)
            time.sleep(1)
            self.rxarm.gripper.grasp()
            time.sleep(0.5)
            self.move_to(small_list[i][0], small_list[i][1], small_list[i][2] + 0.1, -np.pi/2)
            time.sleep(0.5)
            self.move_to(small_drop_list[i][0], small_drop_list[i][1], small_drop_list[i][2] + 0.1, -np.pi/2)
            time.sleep(0.5)
            self.move_to(small_drop_list[i][0], small_drop_list[i][1], small_drop_list[i][2] + 0.02, -np.pi/2)
            time.sleep(1)
            self.rxarm.gripper.release()
            time.sleep(0.5)
            self.move_to(small_drop_list[i][0], small_drop_list[i][1], small_drop_list[i][2] + 0.1, -np.pi/2)
            time.sleep(0.5)

        for i in range(len(large_list)):
            self.move_to(large_list[i][0], large_list[i][1], large_list[i][2] + 0.1, -np.pi/2)
            time.sleep(0.5)
            self.move_to(large_list[i][0], large_list[i][1], large_list[i][2] - 0.005, -np.pi/2)
            time.sleep(1)
            self.rxarm.gripper.grasp()
            time.sleep(0.5)
            self.move_to(large_list[i][0], large_list[i][1], large_list[i][2] + 0.1, -np.pi/2)
            time.sleep(0.5)
            self.move_to(large_drop_list[i][0], large_drop_list[i][1], large_drop_list[i][2] + 0.1, -np.pi/2)
            time.sleep(0.5)
            self.move_to(large_drop_list[i][0], large_drop_list[i][1], large_drop_list[i][2] + 0.02, -np.pi/2)
            time.sleep(1)
            self.rxarm.gripper.release()
            time.sleep(0.5)
            self.move_to(large_drop_list[i][0], large_drop_list[i][1], large_drop_list[i][2] + 0.1, -np.pi/2)
            time.sleep(0.5)
            
        time.sleep(2)
        self.initialize_rxarm()

        self.next_state = "idle"

    def event2(self):
        self.current_state = "event2"

        small_list = np.array([[150, 275, 16], [250, 125, 13], [250, 225, 13]]) * 0.001
        large_list = np.array([[0, 275, 35], [-200, 175, 32], [-250, 75, 28]]) * 0.001

        small_drop_list = np.array([[250, -25, 16]]) * 0.001
        large_drop_list = np.array([[-250, -25, 32]]) * 0.001

        for i in range(3):
            self.move_to(large_list[i][0], large_list[i][1], large_list[i][2] + 0.1, -np.pi/2)
            time.sleep(0.5)
            self.move_to(large_list[i][0], large_list[i][1], large_list[i][2] - 0.01, -np.pi/2)
            time.sleep(0.5)
            self.rxarm.gripper.grasp()
            time.sleep(0.5)
            self.move_to(large_list[i][0], large_list[i][1], large_list[i][2] + 0.1, -np.pi/2)
            time.sleep(0.5)
            self.move_to(large_drop_list[0][0], large_drop_list[0][1], i*0.017 + 0.1, -np.pi/2)
            time.sleep(0.5)
            self.move_to(large_drop_list[0][0], large_drop_list[0][1], i*0.032 + 0.037, -np.pi/2)
            time.sleep(0.5)
            self.rxarm.gripper.release()
            time.sleep(1)
            self.move_to(large_drop_list[0][0], large_drop_list[0][1], i*0.02 + 0.1, -np.pi/2)
            time.sleep(0.5)
            
        for i in range(3):
            self.move_to(small_list[i][0], small_list[i][1], small_list[i][2] + 0.1, -np.pi/2)
            time.sleep(0.5)
            self.move_to(small_list[i][0], small_list[i][1], small_list[i][2] + 0.005, -np.pi/2)
            time.sleep(0.5)
            self.rxarm.gripper.grasp()
            time.sleep(0.5)
            self.move_to(small_list[i][0], small_list[i][1], small_list[i][2] + 0.1, -np.pi/2)
            time.sleep(0.5)
            self.move_to(small_drop_list[0][0], small_drop_list[0][1],  i*0.008 +  0.1, -np.pi/2)
            time.sleep(0.5)
            self.move_to(small_drop_list[0][0], small_drop_list[0][1],  i*0.018 + 0.026, -np.pi/2)
            time.sleep(0.5)
            self.rxarm.gripper.release()
            time.sleep(1)
            self.move_to(small_drop_list[0][0], small_drop_list[0][1], i*0.008 + 0.1, -np.pi/2)
            time.sleep(0.5)

        time.sleep(2)
        self.initialize_rxarm()

        self.next_state = "idle"

    def compare(self):
        self.current_state = "compare"
        self.rxarm.compare_FKIK()
        self.next_state = "idle"

    def detect(self):
        self.current_state = "detect"
        self.camera.blockDetector()

        self.next_state = "idle"



    def calibrate(self):
        """!
        @brief      Gets the user input to perform the calibration
        """
        self.current_state = "calibrate"
        # from camera.py
        top_left = np.array([383, 233])
        bottom_right = np.array([886, 535])
        
        dest_pts = np.array([bottom_right[0], bottom_right[1], 
                 bottom_right[0], top_left[1],
                 top_left[0], top_left[1],
                 top_left[0], bottom_right[1]]).reshape((4, 2))
        
        
        self.camera.H = cv2.findHomography(self.camera.src_pts, dest_pts)[0]
        # print("Homography_matrix: ", self.camera.H)

        # extrinsic matrix from cv2 PnP
        intrinsic_matrix = np.array([[900.18, 0.0, 661.89],[0.0, 900.52, 367.95],[0.0, 0.0, 1.0]])

        tags_cords = np.array([[250, -25, 0],
                               [250, 275, 0],
                               [-250, 275, 0],
                               [-250, -25, 0]], dtype=np.float64)
        
        image_cords = []
        for tag in self.camera.tag_detections.detections:
            image_cords.append([int(tag.centre.x), int(tag.centre.y)])
        image_cords = np.array(image_cords, dtype=np.float64)

        _, rvec, tvec = cv2.solvePnP(tags_cords, image_cords, intrinsic_matrix, self.camera.distortion_coeffs)
        rotation_matrix, _ = cv2.Rodrigues(rvec)

        self.camera.auto_extrinsic[:3][:] = np.concatenate((rotation_matrix, tvec), axis=1)
        self.camera.auto_extrinsic[3][3] = 1

        self.camera.H_try = np.block([
            [rotation_matrix, tvec],
            [np.array([0,0,0,1.0])]
        ])

        self.camera.H_inv = np.block([
            [rotation_matrix.T, -rotation_matrix.T@tvec],
            [np.array([0,0,0,1.0])]
        ])
        K = np.zeros_like(self.camera.offset)

        
        for i in range(720):
            for j in range(1280):
                K[i][j] = int((i + j)/91)
        self.camera.offset = K        
        
        self.status_message = "Automatic Calibration is completed"
        self.next_state = "idle"

    def initialize_rxarm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.current_state = "initialize_rxarm"
        self.status_message = "RXArm Initialized!"
        if not self.rxarm.initialize():
            print('Failed to initialize the rxarm')
            self.status_message = "State: Failed to initialize the rxarm!"
            time.sleep(5)
        self.next_state = "idle"

class StateMachineThread(QThread):
    """!
    @brief      Runs the state machine
    """
    updateStatusMessage = pyqtSignal(str)
    
    def __init__(self, state_machine, parent=None):
        """!
        @brief      Constructs a new instance.

        @param      state_machine  The state machine
        @param      parent         The parent
        """
        QThread.__init__(self, parent=parent)
        self.sm=state_machine

    def run(self):
        """!
        @brief      Update the state machine at a set rate
        """
        while True:
            self.sm.run()
            self.updateStatusMessage.emit(self.sm.status_message)
            time.sleep(0.05)