import cv2
import numpy as np
import torch
from camera import CameraRos,CameraNotRos,CameraAirSim
import random
IMG_WIDTH= 640
IMG_HEIGHT = 480
CAMERA_NOT_ROS_MODE = 'NOT_ROS'
CAMERA_ROS_MODE = 'ROS'
CAMERA_ROS_AIRSIM = 'AIRSIM'

class detector:
	def __init__(self,IP='10.13.4.174',MODE='AIRSIM'):
		self.model = torch.hub.load('ultralytics/yolov5', 'yolov5l6')
		self.model.conf = 0.5
		self.camera = None
		self.camera_mode = MODE
		self.server = None
		self.IP = IP
		if self.camera_mode == CAMERA_ROS_MODE:
			self.camera = CameraRos()
		elif self.camera_mode == CAMERA_NOT_ROS_MODE:
			self.camera = CameraNotRos()
		else:
			self.camera = CameraAirSim()

		# self.color = None
		# self.depth = None

	def initialize_detector(self):
		self.start_camera()

	def get_image_size(self):
		return IMG_WIDTH,IMG_HEIGHT

	def start_camera(self):
		self.camera.start(self.IP)
		# if self.server:
		# 	self.server.start_server()

	def close_camera(self):
		self.camera.close()
		# if self.server:
		# 	self.server.stop_server()

	def get_mid_pos(self, frame, box, depth_data, randnum=24): #,
		distance_list = []
		mid_pos = [(box[0] + box[2]) // 2, (box[1] + box[3]) // 2]  # 确定索引深度的中心像素位置
		min_val = min(abs(box[2] - box[0]), abs(box[3] - box[1]))  # 确定深度搜索范围
		# print(box,)
		for i in range(randnum):
			bias = random.randint(-min_val // 4, min_val // 4)
			dist = depth_data[int(mid_pos[1] + bias), int(mid_pos[0] + bias)]
			cv2.circle(frame, (int(mid_pos[0] + bias), int(mid_pos[1] + bias)), 4, (255, 0, 0), -1)
			# print(int(mid_pos[1] + bias), int(mid_pos[0] + bias))
			if dist:
				distance_list.append(dist)
		distance_list = np.array(distance_list)
		distance_list = np.sort(distance_list)[randnum // 2 - randnum // 4:randnum // 2 + randnum // 4]  # 冒泡排序+中值滤波
		# print(distance_list, np.mean(distance_list))
		return np.mean(distance_list)

	def dectshow(self,org_img, boxs, depth_data):
		img = org_img.copy()
		for box in boxs:
			cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
			dist = self.get_mid_pos(org_img, box, depth_data, 24)
			cv2.putText(img, box[-1] + str(dist / 1000)[:4] + 'm',
						(int(box[0]), int(box[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
		cv2.imshow('dec_img', img)

	def get_detections(self):
		color,depth = self.camera.getColorDepth()

		# results = selfmodel(color_image)
		yolo_detection = self.model(color)
		results = yolo_detection.pandas()
		return results,color,depth

