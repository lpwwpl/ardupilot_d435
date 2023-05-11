import airsim
import time
import cv2
import numpy as np
import os

def capture_image(client):
    #function to capture an image
    png_image = client.simGetImage("front_center_custom", airsim.ImageType.Scene)
    image = cv2.imdecode(airsim.string_to_uint8_array(png_image), cv2.IMREAD_UNCHANGED)
    return image


client = airsim.MultirotorClient()  # connect to the AirSim simulator
client.confirmConnection()
value = client.enableApiControl(True)       # 获取控制权
value = client.armDisarm(True)              # 解锁
value = client.takeoffAsync().join()        # 第一阶段：起飞
# client.moveToPositionAsync(52, -52, -60, 5).join()
# value = client.moveToPositionAsync(-10, 10, -10, 5).join()
responses = client.simGetImages([
    airsim.ImageRequest(1, airsim.ImageType.Scene, False, False),
    airsim.ImageRequest("1", airsim.ImageType.DepthPlanar, True)])
print('Retrieved images: {}'.format(len(responses)))


# imgcolor = np.fromstring(responses[0].image_data_uint8, dtype=np.uint8)
# img_rgb = imgcolor.reshape(responses[0].height, responses[0].width, 3)
# # img_rgb = np.flipud(img_rgb)
# cv2.imshow("imgcolor", img_rgb)
#
#
#
# imgdepth = airsim.get_pfm_array(responses[1])
# imgdepth = np.flipud(imgdepth)
# airsim.write_pfm(os.path.normpath( 'test.pfm'), imgdepth)
# depth_img = cv2.imread('test.pfm', flags=-1)
# cv2.imshow("imgdepth", depth_img)
# cv2.waitKey(1)

client.moveToZAsync(-2, 1).join()   # 第二阶段：上升到2米高度
# client.moveToZAsync(2, 1).join()   # 第二阶段：上升到2米高度
 # 飞正方形
client.moveByVelocityZAsync(1, 0, -4, 8).join()     # 第三阶段：以1m/s速度向前飞8秒钟
client.moveByVelocityZAsync(0, 1, -4, 8).join()     # 第三阶段：以1m/s速度向右飞8秒钟
client.moveByVelocityZAsync(-1, 0, -4, 8).join()    # 第三阶段：以1m/s速度向后飞8秒钟
client.moveByVelocityZAsync(0, -1, -4, 8).join()    # 第三阶段：以1m/s速度向左飞8秒钟

 # 悬停 2 秒钟
# client.hoverAsync().join()          # 第四阶段：悬停6秒钟
time.sleep(6)

client.landAsync().join()           # 第五阶段：降落
client.armDisarm(False)             # 上锁
client.enableApiControl(False)      # 释放控制权