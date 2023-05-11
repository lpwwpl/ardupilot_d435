import pyrealsense2 as rs
import roslibpy
import time
import cv2
import logging
import base64

import numpy as np
from PIL import Image
# import air_sim
import airsim
class BaseCamera:
    def __init__(self):
        pass

    def start(self, ip=''):
        pass

    def stop(self):
        pass

    def getColorDepth(self):
        return None


    def getRGBImg(self):
        return None

    def getDepthImg(self):
        return None

class CameraRos(BaseCamera):
    def __init__(self):
        # Configure logging
        fmt = '%(asctime)s %(levelname)8s: %(message)s'
        logging.basicConfig(format=fmt, level=logging.INFO)
        self.log = logging.getLogger(__name__)

    def start(self, ip):
        self.client = roslibpy.Ros(host=ip, port=9090)
        self.client.run()
        print('Is ROS connected?', self.client.is_connected)

    def close(self):
        self.client.terminate()

    def callback(self, msg):
        self.log.info('Received image seq=%d', msg['header']['seq'])
        base64_bytes = msg['data'].encode('ascii')
        image_bytes = base64.b64decode(base64_bytes)
        return image_bytes

    def getColorDepth(self):
        roslibpy.subscribe_topic("/camera/depth/image_rect_raw/compressed", 'sensor_msgs/CompressedImage')
        roslibpy.subscribe_topic("/camera/color/image_rect_color/compressed", 'sensor_msgs/CompressedImage')

        return None,None

    def getRGBImg(self):
        roslibpy.subscribe_topic("/camera/color/image_rect_color/compressed", 'sensor_msgs/CompressedImage')

        return None

    def getDepthImg(self):
        roslibpy.subscribe_topic("/camera/depth/image_rect_raw/compressed", 'sensor_msgs/CompressedImage')
        return None

    def subscribe_topic(self,topic_name, msg_type, compressed=False):
        listener = roslibpy.Topic(self.client, topic_name, msg_type, throttle_rate=50)
        listener.subscribe(self.callback)
        try:
            while True:
                pass
        except KeyboardInterrupt:
            self.client.terminate()

class CameraNotRos(BaseCamera):
    def __init__(self):
        self.pipeline, self.streamcfg = self.openPipeline()
        self.colorizer = rs.colorizer()
        self.pc = rs.pointcloud()
        self.releaselist = []
        self.stream = None

    def disableAllConfiguration(self):
        self.streamcfg.disable_all_streams()

    def useRGBCamera(self):
        self.streamcfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    def useDepthCamera(self):
        self.streamcfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    def stop(self):
        try:
            self.disableAllConfiguration()
            self.pipeline.stop()
        except:
            pass

    def start(self,IP=''):
        # self.stop()
        self.useRGBCamera()
        self.useDepthCamera()
        pipeline_profile = self.pipeline.start(self.streamcfg)

    def openPipeline(self):
        cfg = rs.config()
        pipeline = rs.pipeline()
        # pipeline_profile = pipeline.start(cfg)
        # sensor = pipeline_profile.get_device().first_depth_sensor()
        return pipeline, cfg

    def getColorDepth(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return color_image,depth_image

    def getRGBImg(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(color_frame.get_data())
        return depth_image

    def getDepthImg(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        return depth_image

    # def getPointCloudStream(self):
    #     pcd, timestamp = self.stream._getStreamAndTimestamp(STREAM_PTC)
    #     frame_data = self.stream.update_frame(pcd, timestamp)
    #     return frame_data



class CameraAirSim():
    def __init__(self):
        self.client = airsim.MultirotorClient()  # connect to the AirSim simulator
        self.client.confirmConnection()

    def start(self, ip=''):
        pass

    def stop(self):
        # self.client.
        pass

    def getColorDepth(self):
        responses = self.client.simGetImages([
            airsim.ImageRequest("front", airsim.ImageType.Scene, False, False),
            airsim.ImageRequest("front", airsim.ImageType.DepthPlanar, True,False)],vehicle_name='Copter')

        imgcolor = np.fromstring(responses[0].image_data_uint8, dtype=np.uint8)
        img_rgb = imgcolor.reshape(responses[0].height, responses[0].width, 3)

        imgdepth = airsim.get_pfm_array(responses[1])
        imgdepth = np.flipud(imgdepth)
        imgdepth = imgdepth*10
        return img_rgb,imgdepth
    # def getColorDepth(self):
    #     global client
    #     responses = self.client.simGetImages([
    #         airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
    #         airsim.ImageRequest("1", airsim.ImageType.DepthVis, True,False)])
    #
    #     imgcolor = np.fromstring(responses[0].image_data_uint8, dtype=np.uint8)
    #     img_rgb = imgcolor.reshape(responses[0].height, responses[0].width, 3)
    #
    #     img_1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
    #     img_depthvis_bgr = img_1d.reshape(responses[0].height, responses[0].width, 3)
    #
    #     return img_rgb,img_depthvis_bgr

    def getRGBImg(self):
        responses = self.client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
        print('Retrieved images: %d', len(responses))

        imgcolor = np.fromstring(responses[0].image_data_uint8, dtype=np.uint8)
        img_rgb = imgcolor.reshape(responses[0].height, responses[0].width, 3)
        # # img_rgb = np.flipud(img_rgb)
        # cv2.imshow("imgcolor", img_rgb)

        return img_rgb

    # def getDepthImg(self):
    #     responses = self.client.simGetImages([
    #         airsim.ImageRequest("0", airsim.ImageType.DepthPlanar, True)])
    #     print('Retrieved images: %d', len(responses))
    #     imgdepth = airsim.get_pfm_array(responses[1])
    #     imgdepth = np.flipud(imgdepth)
    #     # airsim.write_pfm(os.path.normpath( 'test.pfm'), imgdepth)
    #     # depth_img = cv2.imread('test.pfm', flags=-1)
    #     return imgdepth
    def getDepthImg(self):
        responses = self.client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.DepthVis, True)])
        print('Retrieved images: %d', len(responses))
        img_1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        img_depthvis_bgr = img_1d.reshape(responses[0].height, responses[0].width, 3)
        # airsim.write_pfm(os.path.normpath( 'test.pfm'), imgdepth)
        # depth_img = cv2.imread('test.pfm', flags=-1)
        return img_depthvis_bgr

