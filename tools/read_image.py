import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.last_saved_time = time.time()
        self.save_interval = 5  # 保存间隔，单位为秒
        self.image_count = 0

    def image_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_saved_time >= self.save_interval:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            cv_image = cv2.resize(cv_image, (320, 180), interpolation=cv2.INTER_AREA)
            image_path = '/home/unitree/unitree_ros2/image.png'
            cv2.imwrite(image_path, cv_image)
            rospy.loginfo(f"Saved image: {image_path}")
            self.image_count += 1
            self.last_saved_time = current_time

def main():
    rospy.init_node('image_saver', anonymous=True)
    image_saver = ImageSaver()
    rospy.Subscriber('/camera/infra1/image_rect_raw', Image, image_saver.image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
