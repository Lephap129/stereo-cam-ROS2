# disparity_viewer.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import DisparityImage
import cv2
import numpy as np

class DisparityViewer(Node):
    def __init__(self):
        super().__init__('disparity_viewer')
        self.subscription = self.create_subscription(
            DisparityImage,
            '/disparity',  # Change this to your topic name
            self.listener_callback,
            10
        )
        self.window_name = 'Disparity Image'
        cv2.namedWindow(self.window_name)

    def listener_callback(self, msg):
        # Convert DisparityImage to OpenCV format
        # Extract the image data from the message
        width = msg.width
        height = msg.height
        image_data = np.frombuffer(msg.image.data, np.float32).reshape((height, width))

        # Normalize the disparity image for visualization
        cv2.normalize(image_data, image_data, 0, 255, cv2.NORM_MINMAX)
        image_data = np.uint8(image_data)

        # Display the image
        cv2.imshow(self.window_name, image_data)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    disparity_viewer = DisparityViewer()
    rclpy.spin(disparity_viewer)
    disparity_viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
