# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

import rclpy.time
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
import cv2
import numpy as np
import time

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.left_image_raw = self.create_publisher(Image, 'left/image_rect', 10)
        self.left_image_info = self.create_publisher(CameraInfo, 'left/camera_info', 10)
        self.right_image_raw = self.create_publisher(Image, 'right/image_rect', 10)
        self.right_image_info = self.create_publisher(CameraInfo, 'right/camera_info', 10)
        self.cam_left_info = CameraInfo()
        self.cam_right_info = CameraInfo()
        self.cam_left_image = Image()
        self.cam_right_image = Image()
        self.url = "http://192.168.1.34:8000/video-stream"
        # Open the video stream
        self.cap = cv2.VideoCapture(self.url)
        self.i = 0
        self.fps = 0
        prev_frame_time = 0
        
        # Left camera parameters
        self.left_camera_matrix = np.array([[575.5067940856015, 0.0, 136.92275588744678],
                                            [0.0, 573.0696844735534, 114.30618571525305],
                                            [0.0, 0.0, 1.0]])
        self.left_distortion_coeffs = np.array([0.09064540330305361, -0.28177389168100264, 0.002227138811270843, -0.010179409105293118, 0.0])
        self.left_rectification_matrix = np.array([[0.999943415200907, 0.00880959686525453, -0.005963002550570933],
                                                  [-0.008786587185104356, 0.9999538929348181, 0.0038740018250803367],
                                                  [0.005996856008357867, -0.0038213881736697936, 0.9999747170356065]])
        self.left_projection_matrix = np.array([[615.4034350673139, 0.0, 140.80093002319336, 0.0],
                                                 [0.0, 615.4034350673139, 107.71150207519531, 0.0],
                                                 [0.0, 0.0, 1.0, 0.0]])

        # Right camera parameters
        self.right_camera_matrix = np.array([[579.3037221300491, 0.0, 146.52005086645758],
                                             [0.0, 578.0868402952449, 101.15072294187623],
                                             [0.0, 0.0, 1.0]])
        self.right_distortion_coeffs = np.array([0.04286397933931401, -0.0486072247147266, -0.0044226692949588245, -0.016649630944173304, 0.0])
        self.right_rectification_matrix = np.array([[0.9999915422448469, 0.002259356114384297, 0.0034366769881782975],
                                                   [-0.0022461158892835872, 0.9999900599548895, -0.0038516171705365353],
                                                   [-0.0034453450022585296, 0.003843865419712075, 0.9999866770594749]])
        self.right_projection_matrix = np.array([[615.4034350673139, 0.0, 140.80093002319336, -203.7009411926052],
                                                  [0.0, 615.4034350673139, 107.71150207519531, 0.0],
                                                  [0.0, 0.0, 1.0, 0.0]])

        # Extrinsic parameters (assuming left camera as reference)
        self.T = np.array([-0.33100110713168046, -0.0007478557004509127, -0.0011375535090889718])
        self.R = np.array([[0.9999340323795456, 0.006576676029019467, -0.00941691146773936],
                           [-0.00650422046942552, 0.9999491684627007, 0.007704259006459225],
                           [0.009467101207181973, -0.007642501106497973, 0.9999259803463305]])
        while True:
            msg = String()
            # Read a frame from the stream
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to retrieve frame")
                self.get_logger().error("Failed to retrieve frame")
                return None
            # frame = cv2.imread("src/stereo_cam/stereo_cam/stream.jpg")
            frameL, frameR = self.preprocess_frame(frame)
            # Get the current ROS2 time
            now = Clock().now().to_msg()
            # Left image 
            self.cam_left_info.header.stamp = now
            self.cam_left_info.header.frame_id = "camera_left"
            self.cam_left_info.height = 200
            self.cam_left_info.width = 300
            self.cam_left_info.distortion_model = "plumb_bob"
            self.cam_left_info.d = self.left_distortion_coeffs.tolist()
            self.cam_left_info.k = self.left_camera_matrix.flatten().tolist()
            self.cam_left_info.r = self.left_rectification_matrix.flatten().tolist()
            self.cam_left_info.p = self.left_projection_matrix.flatten().tolist()
            
            self.cam_left_image.header.stamp = now
            self.cam_left_image.header.frame_id = "camera_left"
            self.cam_left_image.height = frameL.shape[0]
            self.cam_left_image.width = frameL.shape[1]
            self.cam_left_image.encoding = "mono8"
            self.cam_left_image.is_bigendian = 0
            self.cam_left_image.step = frameL.shape[1]
            self.cam_left_image.data = np.array(frameL, dtype=np.uint8).tobytes()
            
            # Right image
            self.cam_right_info.header.stamp = now
            self.cam_right_info.header.frame_id = "camera_right"
            self.cam_right_info.height = 200
            self.cam_right_info.width = 300
            self.cam_right_info.distortion_model = "plumb_bob"
            self.cam_right_info.d = self.right_distortion_coeffs.tolist()
            self.cam_right_info.k = self.right_camera_matrix.flatten().tolist()
            self.cam_right_info.r = self.right_rectification_matrix.flatten().tolist()
            self.cam_right_info.p = self.right_projection_matrix.flatten().tolist()
            
            self.cam_right_image.header.stamp = now
            self.cam_right_image.header.frame_id = "camera_right"
            self.cam_right_image.height = frameR.shape[0]
            self.cam_right_image.width = frameR.shape[1]
            self.cam_right_image.encoding = "mono8"
            self.cam_right_image.is_bigendian = 0
            self.cam_right_image.step = frameR.shape[1]
            self.cam_right_image.data = np.array(frameR, dtype=np.uint8).tobytes()
            
            
            self.left_image_raw.publish(self.cam_left_image)
            self.left_image_info.publish(self.cam_left_info)
            self.right_image_raw.publish(self.cam_right_image)
            self.right_image_info.publish(self.cam_right_info)
            
            new_frame_time = time.time()
            self.fps = 0.9 * self.fps + 0.1 / (new_frame_time - prev_frame_time)
            prev_frame_time = new_frame_time
            msg.data = 'Hello World!: fps = %d' % self.fps
            if self.i > self.fps//2:
                self.get_logger().info('Publishing: "%s"' % msg.data)
                self.i = 0
            self.i += 1
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0
        
    # Preprocessing function
    def preprocess_frame(self,frame):
        width_crop = 320
        height_crop = 200
        height, width, _ = frame.shape
        # Split left, right image
        frameL = frame[:, :width // 2]
        frameR = frame[:, width // 2:]  
        heightL, widthL,_ = frameL.shape
        # Take center image
        center_x = widthL // 2
        center_y = heightL // 2
        # Take position for crop
        start_x = center_x - (width_crop // 2)
        start_y = center_y - (height_crop // 2)
        # Take crop image
        frameL = frameL[start_y:start_y + height_crop, start_x:start_x + width_crop]
        frameR = frameR[start_y:start_y + height_crop, start_x:start_x + width_crop]
        grayL= cv2.cvtColor(frameL,cv2.COLOR_BGR2GRAY)
        grayR= cv2.cvtColor(frameR,cv2.COLOR_BGR2GRAY)
        return grayL,grayR
    
    def timer_callback(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
