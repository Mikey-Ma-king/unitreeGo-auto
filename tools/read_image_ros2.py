import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()
        self.last_saved_time = time.time()
        self.save_interval = 5  # 保存间隔，单位为秒
        self.image_count = 0

        # 创建订阅者
        self.subscription = self.create_subscription(
            Image,
            '/camera/infra1/image_rect_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_saved_time >= self.save_interval:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            cv_image = cv2.resize(cv_image, (320, 180), interpolation=cv2.INTER_NEAREST)
            image_path = '/home/unitree/unitree_ros2/image1.png'
            cv2.imwrite(image_path, cv_image)
            self.get_logger().info(f"Saved image: {image_path}")
            self.image_count += 1
            self.last_saved_time = current_time

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()

    rclpy.spin(image_saver)

    # 清理并关闭节点
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
