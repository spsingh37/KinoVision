#!/usr/bin/env python3

"""!
Class to represent the camera.
"""
 
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

import cv2
import time
import numpy as np
import numpy.linalg as LA
from PyQt5.QtGui import QImage
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from apriltag_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from detect_block import label_blocks


class Camera():
    """!
    @brief      This class describes a camera.
    """

    def __init__(self):
        """!
        @brief      Construcfalsets a new instance.
        """
        self.VideoFrame = np.zeros((720,1280, 3)).astype(np.uint8)
        self.GridFrame = np.zeros((720,1280, 3)).astype(np.uint8)
        self.BlockFrame = np.zeros((720,1280,3)).astype(np.uint8)
        self.TagImageFrame = np.zeros((720,1280, 3)).astype(np.uint8)
        self.DepthFrameRaw = np.zeros((720,1280)).astype(np.uint16)
        """ Extra arrays for colormaping the depth image"""
        self.DepthFrameHSV = np.zeros((720,1280, 3)).astype(np.uint8)
        self.DepthFrameRGB = np.zeros((720,1280, 3)).astype(np.uint8)
        self.frame_count = 0
        self.offset = np.zeros((720,1280)).astype(np.uint8)
        
        
        
        


        # mouse clicks & calibration variables
        self.cameraCalibrated = True ##
        self.intrinsic_matrix = np.array([[900.18 , 0, 661.89],[0, 900.52, 367.95],[0, 0, 1 ]]) ##
        
        # for Task 2.2, automatic calibration
        self.auto_extrinsic = np.eye(4) 
        self.distortion_coeffs = np.array([0.13614392280578613,
                                          -0.46406102180480957,
                                        0.00012681944645009935,
                                        0.00037506374064832926,
                                            0.4243681728839874])

        self.last_click = np.array([0, 0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5, 2), int)
        self.depth_click_points = np.zeros((5, 2), int)
        self.grid_x_points = np.arange(-450, 500, 50)
        self.grid_y_points = np.arange(-175, 525, 50)
        self.grid_points = np.array(np.meshgrid(self.grid_x_points, self.grid_y_points))
        self.tag_detections = np.array([])
        self.tag_locations = [[-250, -25], [250, -25], [250, 275]]
        """ block info """
        self.block_contours = np.array([])
        self.block_detections = np.array([])
        self.detections = {}
        self.H = np.array([[-1,0,1269],[0,-1,768],[0,0,1]],dtype=np.float32)
        self.H_try = np.eye(4)
        self.H_inv = np.eye(4)
        self.prev_depth = np.zeros((720,1280))

        self.src_pts = np.array([[383,233],[383,535],[886,535],[886,233]])    

    def loadVideoFrame(self):
        """!
        @brief      Loads a video frame.
        """
        self.VideoFrame = cv2.cvtColor(
            cv2.imread("data/rgb_image.png", cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB)

        

    def loadDepthFrame(self):
        """!
        @brief      Loads a depth frame.
        """
        self.DepthFrameRaw = cv2.imread("data/raw_depth.png",
                                        0).astype(np.uint16)
        ####### Test done here
        
    def convertQtVideoFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.VideoFrame, (1280, 720))
            frame = cv2.warpPerspective(frame, self.H, (frame.shape[1], frame.shape[0]))
            # self.frame_count += 1
            # if self.frame_count % 5 == 0:
            #     frame = self.blockDetector(frame)
            #frame = self.blockDetector(frame)
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtGridFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.GridFrame, (1280, 720))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            

            return img
        except:
            return None

    def convertQtDepthFrame(self):
        """!
       @brief      Converts colormaped depth frame to format suitable for Qt

       @return     QImage
       """
        try:
            frame = self.DepthFrameRGB 
            frame = cv2.warpPerspective(frame, self.H, (frame.shape[1], frame.shape[0]))
            img = QImage(frame, frame.shape[1],
                         frame.shape[0], QImage.Format_RGB888)
            return img
        except:
            return None
    
    def convertQtBlockFrame(self):
        """!
       @brief      Converts colormaped depth frame to format suitable for Qt

       @return     QImage
       """
        try:
            frame = self.BlockFrame
            frame = cv2.warpPerspective(frame, self.H, (frame.shape[1], frame.shape[0]))
            #frame = self.blockDetector(frame)  ## uncomment this line to see the block detection
            img = QImage(frame, frame.shape[1],
                         frame.shape[0], QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtTagImageFrame(self):
        """!
        @brief      Converts tag image frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.TagImageFrame, (1280, 720))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def getAffineTransform(self, coord1, coord2):
        """!
        @brief      Find the affine matrix transform between 2 sets of corresponding coordinates.

        @param      coord1  Points in coordinate frame 1
        @param      coord2  Points in coordinate frame 2

        @return     Affine transform between coordinates.
        """
        pts1 = coord1[0:3].astype(np.float32)
        pts2 = coord2[0:3].astype(np.float32)
        # print(cv2.getAffineTransform(pts1, pts2))
        return cv2.getAffineTransform(pts1, pts2)

    def loadCameraCalibration(self, file):
        """!
        @brief      Load camera intrinsic matrix from file.

                    TODO: use this to load in any calibration files you need to

        @param      file  The file
        """
        pass
    def VideoBlock(self):
        """!
        @brief      Process a video frame
        """
        
        for label in self.detections.keys():
            pos = self.detections[label][0]
            # print("pos",pos)
            # pos = np.matmul(np.linalg.inv(self.H), np.array([pos[0][0], pos[0][1], 1]).T)
            # print("new_pos",pos)
            # pos = np.round(pos).astype(int)
            # pos = [(pos[0], pos[1])]
            # print("new_new_pos",pos)

            angle = self.detections[label][1]
            size = self.detections[label][2]
            for temp in range(len(pos)): 
                cv2.circle(self.VideoFrame, (pos[temp][0],pos[temp][1]), 3, (255, 255, 255), -1)
                cv2.rectangle(self.VideoFrame, (pos[temp][0]-25,pos[temp][1]-25), (pos[temp][0]+25,pos[temp][1]+25), color=(0,0,255),thickness=2)

                cv2.putText(
                    self.VideoFrame, label, 
                    (pos[temp][0] - 20, pos[temp][1] - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                    (255, 255, 255), 2
                )
                cv2.putText(
                    self.VideoFrame, size[temp], 
                    (pos[temp][0] - 20, pos[temp][1] - 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                    (255, 255, 255), 2
                )

    def DepthConverter(self):
        """!
        @brief Converts frame to colormaped formats in HSV and RGB
        """

        z = self.DepthFrameRaw
        x,y = np.meshgrid(range(1280), range(720), indexing='xy')
        x,y,z = x.flatten(), y.flatten(), z.flatten()
        xy_c = np.stack([x,y,np.ones_like(x)], axis=0)
        self.intrinsic_matrix_inv = LA.inv(self.intrinsic_matrix)
        world_c = self.intrinsic_matrix_inv @ xy_c * z
        
        world_c = np.concatenate([world_c, np.ones_like(z).reshape(1,-1)], axis=0)
        world_w = self.H_inv @ world_c
        depth = world_w[2].reshape(720, 1280)
        beta = 0.2
        depth = beta * self.prev_depth + (1-beta) * depth
        self.prev_depth = depth
        depth = depth + self.offset

        
        self.DepthFrameRGB[..., 0] = depth
        self.DepthFrameRGB[..., 1] = depth
        self.DepthFrameRGB[..., 2] = depth
    


    def world_cood(self, cx, cy):
        old_u = cx
        old_v = cy
        old = np.array([[old_u],[old_v],[1]])
        new2 = np.dot(np.linalg.inv(self.H),old)
        x = int(new2[0][0]/new2[2][0])
        y = int(new2[1][0]/new2[2][0])
        z = int(new2[2]/new2[2])

        x = max(100, min(x, 1170))
        y = max(25, min(y, 700))

        d = self.DepthFrameRaw[y][x]
    
        intrix = self.intrinsic_matrix
        extrix = self.auto_extrinsic

        proj_mat = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]])
        proj_mat = np.dot(intrix,proj_mat)
    
        mouse_mat = np.array([[x],[y],[1]])
        intrix_cal = d * np.linalg.inv(intrix)
        cam_coord = np.ones((4,1))
        cam_coord[:3] = np.dot(intrix_cal,mouse_mat)
        # print("extrix",extrix)
        #camera frame to world frame
        world_coord = np.dot(np.linalg.inv(extrix),cam_coord)
        return world_coord[:3]

    def blockDetector(self):
        """!
        @brief      Detect blocks from rgb
        

                    TODO: Implement your block detector here. You will need to locate blocks in 3D space and put their XYZ
                    locations in self.block_detections
        """
        colors = {}
        colors['red'] = [   ((0,150,50),   (9,255,255)), 
                            ((170,150,50), (179,255,255))]
        colors['orange'] =  ((7,150,100),   (16,255,255))
        colors['yellow'] =  ((20,180,100),  (29,255,255))
        colors['green'] =   ((60,100,50),   (89,255,255))
        colors['blue'] =    ((100,150,80),  (111,255,255))
        colors['violet'] =  ((113,82,51),   (151,255,255))


        temp_image = self.VideoFrame.copy()
        hsv_image = cv2.cvtColor(temp_image, cv2.COLOR_RGB2HSV)
        depth_image = self.DepthFrameRGB.copy()
        depth_detections = self.detectBlocksInDepthImage()
        
        # temp_image = cv2.resize(self.VideoFrame, (1280, 720))
        # temp_image = cv2.warpPerspective(temp_image, self.H, (temp_image.shape[1], temp_image.shape[0]))
        # hsv_image = cv2.cvtColor(temp_image, cv2.COLOR_RGB2HSV)
        # depth_image = self.DepthFrameRGB

        # depth_image = cv2.warpPerspective(depth_image, self.H, (depth_image.shape[1], depth_image.shape[0]))
        # depth_detections = self.detectBlocksInDepthImage()

        hsv_image[:80, 1] = 0
        hsv_image[720-50:, 1] = 0
        hsv_image[:, :200, 1] = 0
        hsv_image[:, 1190:, 1] = 0

        hsv_detections = {}
        hsv_detections = { k:[] for k in colors.keys() }
        for label, hsv_range in colors.items():
            if isinstance(hsv_range, list):
                low, high = hsv_range[0]
                thresh = cv2.inRange(hsv_image, low, high)
                low, high = hsv_range[1]    
                thresh = thresh | cv2.inRange(hsv_image, low, high)
            else:
                low, high = hsv_range
                thresh = cv2.inRange(hsv_image, low, high)
            contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours = self.filter_contour(contours, 150, 4000)
            
            if len(contours) > 0:
                hsv_detections[label] += contours
        checker = []  

        for label in hsv_detections.keys():
            con = hsv_detections[label]
            for k in range(len(con)):
                c = np.array(self.contour_center(con[k]))
                if (c[0] < 275 or c[0] > 1120 or c[1] < 92 or c[1] > 640):
                    continue
                if (c[0]>626 and c[0]<760 and c[1]>420):
                    continue
                if self.is_close(c,checker,40):
                    continue
                checker.append(self.contour_center(con[k])) 
        # print(checker)
        
        self.detections = {}
        for label in hsv_detections.keys():
            c = hsv_detections[label]
            depth = np.array([])
            angle = np.array([])
            size_block_arr = np.array([])
            pos = []
            for contours in c:

                temp = np.array(self.contour_center(contours))
                if (temp[0] < 275 or temp[0] > 1120 or temp[1] < 92 or temp[1] > 640):
                    continue
                if self.contour_center(contours) not in checker:
                    continue
                pos.append(self.contour_center(contours))
                x,y = self.contour_center(contours) 
                z = depth_image[y][x]
                xyz_w = self.world_coordinates(x,y,z)
                depth = np.append(depth,xyz_w[2])

                rect = cv2.minAreaRect(contours)
                angle = np.append(angle,rect[2])
                size_block = ""
                if (cv2.contourArea(contours) > 900):
                    size_block = "big"
                else:
                    size_block = "small"
                size_block_arr = np.append(size_block_arr,size_block)

            world_pos = np.zeros((len(pos),3))
            
            for i in range(len(pos)):
                # print("pos", pos[i])
                # # pos[i][0] = new_pos[0]
                # # pos[i][1] = new_pos[1]
                # pos[i] = (new_pos[0], new_pos[1])
                # print("new_pos", pos)
                pts = np.array([[pos[i][0],pos[i][1]]], np.float32)
                M = self.H
                real_pos = pos.copy()
                ## (n, 1, 2)
                pts1 = pts.reshape(-1,1,2).astype(np.float32)
                dst1 = cv2.perspectiveTransform(pts1, M)
                # print("new_pos",dst1)
                real_pos[i] = (int(dst1[0][0][0]), int(dst1[0][0][1]))
                world_pos[i] = self.world_cood(real_pos[i][0],real_pos[i][1]).flatten()
            self.detections[label] = [pos, angle, size_block_arr, world_pos]
        print(self.detections)
            

    def is_close(self,value, array, tolerance):
        array = np.array(array)
        value = np.array(value)

        for element in array:
            if abs(element[0] - value[0]) <= tolerance and  abs(element[1] - value[1]) <= tolerance:
                
                return True
        return False
    


    ############### comment this if not needed #####################################
    
    def detectBlocksInDepthImage(self):
        """!
        @brief      Detect blocks from depth

                    TODO: Implement a blob detector to find blocks in the depth image

        """
       
        depth_image = self.DepthFrameRGB.copy()
        # depth_image = cv2.warpPerspective(depth_image, self.H, (depth_image.shape[1], depth_image.shape[0]))
        depth = depth_image[..., 0]
        depth = cv2.inRange(depth, 0, 100)
        contours, _ = cv2.findContours(depth, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = self.filter_contour(contours, 100, 2000)
        return contours
                                          
    def projectGridInRGBImage(self):
        """!
        @brief      projects

                    TODO: Use the intrinsic and extrinsic matricies to project the gridpoints 
                    on the board into pixel coordinates. copy self.VideoFrame to self.GridFrame and
                    and draw on self.GridFrame the grid intersection points from self.grid_points
                    (hint: use the cv2.circle function to draw circles on the image)
        """
        modified_image = self.VideoFrame.copy()

        modified_image = cv2.warpPerspective(modified_image, self.H, (modified_image.shape[1], modified_image.shape[0]))

        top_left_1 = np.array([131, 30])
        bottom_right_1 = np.array([1131, 680])
        circle_x = np.arange(top_left_1[0], bottom_right_1[0]+1, 50)
        circle_y = np.arange(top_left_1[1], bottom_right_1[1]+1, 50)
        for x in circle_x:
            for y in circle_y:
                modified_image = cv2.circle(modified_image, (int(x), int(y)), radius=1, color=(0,255,0),thickness=-1)

        self.GridFrame = modified_image

    
    def filter_contour(self,contours, low, high):
        """Filter contours between low and high"""
        return [
            cnt for cnt in contours 
            if cv2.contourArea(cnt) > low
            and cv2.contourArea(cnt) < high
        ]
        
    def contour_center(self,contours):
        M = cv2.moments(contours)
        return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])) 

    def world_coordinates(self, x, y, z):
        
        self.intrinsic_matrix_inv = LA.inv(self.intrinsic_matrix)
        world_c = z * self.intrinsic_matrix_inv @ np.array([x, y, 1])
        world_c = np.append(world_c, 1.0)
        world_w = self.H_inv @ world_c
        return world_w
     
    def drawTagsInRGBImage(self, msg):
        """
        @brief      Draw tags from the tag detection

                    TODO: Use the tag detections output, to draw the corners/center/tagID of
                    the apriltags on the copy of the RGB image. And output the video to self.TagImageFrame.
                    Message type can be found here: /opt/ros/humble/share/apriltag_msgs/msg

                    center of the tag: (detection.centre.x, detection.centre.y) they are floats
                    id of the tag: detection.id
        """
        modified_image = self.VideoFrame.copy()

        for i in range(len(msg.detections)):
            id = msg.detections[i].id 
            center_x = int(msg.detections[i].centre.x)
            center_y = int(msg.detections[i].centre.y)
          
            self.src_pts[i][:] = [center_x,center_y]
            top_left_x = int(msg.detections[i].corners[0].x)
            top_left_y = int(msg.detections[i].corners[0].y)
            bottom_rt_x = int(msg.detections[i].corners[2].x)
            bottom_rt_y = int(msg.detections[i].corners[2].y)
            id_x = center_x+50
            id_y = center_y+50
            bbox_msg = "ID " + str(id)
            modified_image = cv2.circle(modified_image,(center_x,center_y),radius=2,color=(0,255,0),thickness=-1)
            modified_image = cv2.rectangle(modified_image,(top_left_x,top_left_y),(bottom_rt_x,bottom_rt_y),color=(0,0,255),thickness=2)
            modified_image = cv2.putText(modified_image,bbox_msg,(id_x,id_y),cv2.FONT_HERSHEY_SIMPLEX, color=(255, 0, 0), fontScale=1, lineType=cv2.LINE_AA)
        
        self.TagImageFrame = modified_image

    # def transform_image(self, img):
    #     """
    #     Transforms the RGB and depth images to a birdeye view based on precomputed perspective transform matrix
    #     """
    #     dsize = (img.shape[1], img.shape[0])
    #     try:
    #         img = cv2.warpPerspective(img, self.perspective_transform, dsize)
    #     except:
    #         pass
    #     return img

class ImageListener(Node):
    def __init__(self, topic, camera):
        super().__init__('image_listener')
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, topic, self.callback, 10)
        self.camera = camera
        self.frame_cout = 0

    def callback(self, data):
        
        self.frame_cout += 1
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)
        self.camera.VideoFrame = cv_image
        self.camera.VideoBlock()


class TagDetectionListener(Node):
    def __init__(self, topic, camera):
        super().__init__('tag_detection_listener')
        self.topic = topic
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            topic,
            self.callback,
            10
        )
        self.camera = camera

    def callback(self, msg):
        self.camera.tag_detections = msg
        if np.any(self.camera.VideoFrame != 0):
            self.camera.drawTagsInRGBImage(msg)


class CameraInfoListener(Node):
    def __init__(self, topic, camera):
        super().__init__('camera_info_listener')  
        self.topic = topic
        self.tag_sub = self.create_subscription(CameraInfo, topic, self.callback, 10)
        self.camera = camera

    def callback(self, data):
        self.camera.intrinsic_matrix = np.reshape(data.k, (3, 3))


class DepthListener(Node):
    def __init__(self, topic, camera):
        super().__init__('depth_listener')
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, topic, self.callback, 10)
        self.camera = camera

    def callback(self, data):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)
        self.camera.DepthFrameRaw = cv_depth
        self.camera.DepthConverter()



class VideoThread(QThread):
    updateFrame = pyqtSignal(QImage, QImage, QImage, QImage)

    def __init__(self, camera, parent=None):
        QThread.__init__(self, parent=parent)
        self.camera = camera
        image_topic = "/camera/color/image_raw"
        depth_topic = "/camera/aligned_depth_to_color/image_raw"
        camera_info_topic = "/camera/color/camera_info"
        tag_detection_topic = "/detections"
        image_listener = ImageListener(image_topic, self.camera)
        depth_listener = DepthListener(depth_topic, self.camera)
        camera_info_listener = CameraInfoListener(camera_info_topic,
                                                  self.camera)
        tag_detection_listener = TagDetectionListener(tag_detection_topic,
                                                      self.camera)
        
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(image_listener)
        self.executor.add_node(depth_listener)
        self.executor.add_node(camera_info_listener)
        self.executor.add_node(tag_detection_listener)

    def run(self):
        if __name__ == '__main__':
            cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Depth window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Tag window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Grid window", cv2.WINDOW_NORMAL)
            #cv2.namedWindow("Block window", cv2.WINDOW_NORMAL)
            time.sleep(0.5)
        try:
            while rclpy.ok():
                start_time = time.time()
                rgb_frame = self.camera.convertQtVideoFrame()
                depth_frame = self.camera.convertQtDepthFrame()
                tag_frame = self.camera.convertQtTagImageFrame()
                self.camera.projectGridInRGBImage()
                grid_frame = self.camera.convertQtGridFrame()
                #self.camera.detectBlocksInDepthImage()
                #block_frame = self.camera.convertQtBlockFrame()
                if ((rgb_frame != None) & (depth_frame != None)):
                    self.updateFrame.emit(
                        rgb_frame, depth_frame, tag_frame, grid_frame)
                self.executor.spin_once() # comment this out when run this file alone.
                elapsed_time = time.time() - start_time
                sleep_time = max(0.03 - elapsed_time, 0)
                time.sleep(sleep_time)

                if __name__ == '__main__':
                    cv2.imshow(
                        "Image window",
                        cv2.cvtColor(self.camera.VideoFrame, cv2.COLOR_RGB2BGR))
                    cv2.imshow("Depth window", self.camera.DepthFrameRGB)
                    cv2.imshow(
                        "Tag window",
                        cv2.cvtColor(self.camera.TagImageFrame, cv2.COLOR_RGB2BGR))
                    cv2.imshow("Grid window",
                        cv2.cvtColor(self.camera.GridFrame, cv2.COLOR_RGB2BGR))
                    # cv2.imshow("Block window",
                    #     cv2.cvtColor(self.camera.BlockFrame) )
                    
                    
                    cv2.waitKey(3)
                    time.sleep(0.03)
        except KeyboardInterrupt:
            pass
        
        self.executor.shutdown()
        

def main(args=None):
    rclpy.init(args=args)
    try:
        camera = Camera()
        videoThread = VideoThread(camera)
        videoThread.start()
        try:
            videoThread.executor.spin()
        finally:
            videoThread.executor.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()