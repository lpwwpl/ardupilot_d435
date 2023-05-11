import roslibpy
import time
import cv2
import logging
import base64
import numpy as np
from PIL import Image


def bytes2cv(im):
    '''二进制图片转cv2

    :param im: 二进制图片数据，bytes
    :return: cv2图像，numpy.ndarray
    '''
    return cv2.imdecode(np.array(bytearray(im), dtype='uint8'), cv2.IMREAD_UNCHANGED)  # 从二进制图片数据中读取


def cv2bytes(im):
    '''cv2转二进制图片

    :param im: cv2图像，numpy.ndarray
    :return: 二进制图片数据，bytes
    '''
    return np.array(cv2.imencode('.png', im)[1]).tobytes()


class ROS_client_win:
    def __init__(self):
        # Configure logging
        fmt = '%(asctime)s %(levelname)8s: %(message)s'
        logging.basicConfig(format=fmt, level=logging.INFO)
        self.log = logging.getLogger(__name__)


    def connect(self, ip):
        self.client = roslibpy.Ros(host=ip, port=9090)
        self.client.run()
        print('Is ROS connected?', self.client.is_connected)

    def close(self):
        self.client.terminate()


    def public_topic(self, topic_name, msg_type):
        talker = roslibpy.Topic(self.client, topic_name, msg_type)

        while self.client.is_connected:
            talker.publish(roslibpy.Message({'data': 'Hello World!'}))
            print('Sending message...')
            time.sleep(1)

        talker.unadvertise()

    def depth_callback(self, msg):
        self.log.info('Received image seq=%d', msg['header']['seq'])
        base64_bytes = msg['data'].encode('ascii')
        image_bytes = base64.b64decode(base64_bytes)

        img = bytes2cv(image_bytes)
        cv2.imshow('bytes2cv', img)
        if cv2.waitKey(1) == ord('q'):
            exit(-1)

    def color_callback(self, msg):
        self.log.info('Received image seq=%d', msg['header']['seq'])
        base64_bytes = msg['data'].encode('ascii')
        image_bytes = base64.b64decode(base64_bytes)

        img = bytes2cv(image_bytes)
        cv2.imshow('bytes2cv', img)
        if cv2.waitKey(1) == ord('q'):
            exit(-1)

    def subscribe_depth_topic(self,topic_name, msg_type, compressed=False):
        listener = roslibpy.Topic(self.client, topic_name, msg_type, throttle_rate=50)
        listener.subscribe(self.depth_callback)
        try:
            while True:
                pass
        except KeyboardInterrupt:
            self.client.terminate()

    def subscribe_color_topic(self,topic_name, msg_type, compressed=False):
        listener = roslibpy.Topic(self.client, topic_name, msg_type, throttle_rate=50)
        listener.subscribe(self.color_callback)
        try:
            while True:
                pass
        except KeyboardInterrupt:
            self.client.terminate()


if __name__ == "__main__":
    ros_win_client = ROS_client_win()

    ros_win_client.connect("10.13.0.54")

    # ros_win_client.public_topic("/chatter", 'std_msgs/String')

    ros_win_client.subscribe_depth_topic("/camera/depth/image_rect_raw/compressed", 'sensor_msgs/CompressedImage')
    ros_win_client.subscribe_color_topic("/camera/color/image_rect_color/compressed", 'sensor_msgs/CompressedImage')
