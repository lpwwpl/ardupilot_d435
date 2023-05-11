import sys, time
import argparse
import cv2
import collections
from detector import detector

import vision
import keyboard
from control import ArdupilotController



# config
MAX_FOLLOW_DIST = 2                          #meter
MAX_ALT =  2.5                                  #m
MAX_MA_X_LEN = 5
MAX_MA_Z_LEN = 5
MA_X = collections.deque(maxlen=MAX_MA_X_LEN)   #Moving Average X
MA_Z = collections.deque(maxlen=MAX_MA_Z_LEN)   #Moving Average Z


def visualize(img):
   cv2.imshow("out", img)
   cv2.waitKey(1)
   return

def calculate_ma(ma_array):
    sum_ma = 0
    for i in ma_array:
        sum_ma = sum_ma + i

    return sum_ma / len(ma_array)



class Mainflow:
    def __init__(self):
        self.control = ArdupilotController()
        self.detector = detector()
        self.STATE = "takeoff"
        self.setup()
        self.image_width=320
        self.image_height=240
        self.image_center = (self.image_width, self.image_height)

    def setup(self):
        print("setting up detector")
        self.detector.initialize_detector()
        self.image_width, self.image_height = self.detector.get_image_size()
        self.image_center = (self.image_width / 2, self.image_height / 2)

        self.control.configure_PID(args.control)
        self.control.initialize_debug_logs(args.debug_path)

        print("connecting to drone")
        if args.mode == "flight":
            print("MODE = flight")
            self.control.connect_drone('/dev/ttyACM0')
        else:
            print("MODE = test")
            self.control.connect_drone('127.0.0.1:14551')

        self.control.set_flight_altitude(MAX_ALT)

    def run(self):


        while True:
            # main program loop
            """" True or False values depend whether or not
                a PID controller or a P controller will be used  """
            if self.STATE == "track":
                self.control.set_system_state("track")
                self.STATE = self.track()

            elif self.STATE == "search":
                self.control.set_system_state("search")
                self.STATE = self.search()

            elif self.STATE == "takeoff":
                self.STATE = self.takeoff()

            elif self.STATE == "land":
                self.STATE = self.land()

    def prepare_visualisation(self, lidar_distance, person_center, person_to_track, image,
                              yaw_command, x_delta, y_delta, velocity_x_command):
        lidar_vis_x = self.image_width - 50
        lidar_vis_y = self.image_height - 50
        lidar_vis_y2 = int(self.image_height - lidar_distance * 200)
        cv2.line(image, (lidar_vis_x, lidar_vis_y), (lidar_vis_x, lidar_vis_y2), (0, 255, 0), thickness=10, lineType=8,
                 shift=0)
        cv2.putText(image, "distance: " + str(round(lidar_distance, 2)), (self.image_width - 300, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, cv2.LINE_AA)

        # draw path
        cv2.line(image, (int(self.image_center[0]), int(self.image_center[1])), (int(person_center[0]), int(person_center[1])),
                 (255, 0, 0), thickness=10, lineType=8, shift=0)

        # draw bbox around target
        cv2.rectangle(image, (int(person_to_track[0]), int(person_to_track[1])),
                      (int(person_to_track[2]), int(person_to_track[3])), (0, 0, 255), thickness=10)

        # show drone center
        cv2.circle(image, (int(self.image_center[0]), int(self.image_center[1])), 20, (0, 255, 0), thickness=-1, lineType=8,
                   shift=0)

        # show trackable center
        cv2.circle(image, (int(person_center[0]), int(person_center[1])), 20, (0, 0, 255), thickness=-1, lineType=8,
                   shift=0)

        # show stats
        cv2.putText(image, "yaw: " + str(round(yaw_command, 2)) + " forward: " + str(
            round(velocity_x_command, 2)), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, cv2.LINE_AA)
        cv2.putText(image, "x_delta: " + str(round(x_delta, 2)) + " y_delta: " + str(round(y_delta, 2)), (50, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, cv2.LINE_AA)

        visualize(image)

    def takeoff(self):
        self.control.print_drone_report()
        print("State = TAKEOFF -> " + self.STATE)
        self.control.arm_and_takeoff(MAX_ALT)  # start control when drone is ready
        return "search"

    def land(self):
        print("State = LAND -> " + self.STATE)
        self.control.land()
        self.detector.close_camera()
        sys.exit(0)

    def search(self):
        print("State is SEARCH -> " + self.STATE)
        start = time.time()

        self.control.stop_drone()
        while time.time() - start < 40:
            if keyboard.is_pressed('q'):  # if key 'q' is pressed
                print("Closing due to manual interruption")
                self.land()  # Closes the loop and program

            detections,_,_ = self.detector.get_detections()
            print("searching: " + str(len(detections)))
            if len(detections) > 0:
                return "track"
            # if "test" == args.mode:
            #     cv2.putText(image, "searching target. Time left: " + str(40 - (time.time() - start)), (50, 50),
            #                 cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, cv2.LINE_AA)
            #     visualize(image)

        return "land"


    def track(self):
        print("State is TRACKING -> " + self.STATE)
        while True:
            if keyboard.is_pressed('q'):  # if key 'q' is pressed
                print("Closing due to manual interruption")
                self.land()  # Closes the loop and program
            detections,color,depth = self.detector.get_detections()

            if len(detections) > 0:
                obj_to_track = detections.xyxy[0].values[0]  # only track 1 person
                # detector.dectshow(color, obj_to_track, depth)

                ux = int((obj_to_track[0] + obj_to_track[2]) / 2)  # 计算像素坐标系的x
                uy = int((obj_to_track[1] + obj_to_track[3]) / 2)  # 计算像素坐标系的y
                obj_center = (ux,uy)

                x_delta = vision.get_single_axis_delta(self.image_center[0], obj_center[0])  # get x delta
                y_delta = vision.get_single_axis_delta(self.image_center[1], obj_center[1])  # get y delta

                obj_dist = self.detector.get_mid_pos(color,obj_to_track,depth,24)

                MA_Z.append(obj_dist)
                MA_X.append(x_delta)

                # depth x command > PID and moving average
                velocity_z_command = 0
                if obj_dist > 0: # and len(MA_Z) > 0:  # only if a valid lidar value is given change the forward velocity. Otherwise keep previos velocity (done by arducopter itself)
                    z_delta_MA = calculate_ma(MA_Z)
                    z_delta_MA = z_delta_MA - MAX_FOLLOW_DIST
                    self.control.setZDelta(z_delta_MA)
                    velocity_z_command = self.control.getMovementVelocityXCommand()

                # yaw command > PID and moving average
                # yaw_command = 0
                # if len(MA_X) > 0:
                velocity_x_command = 0
                x_delta_MA = calculate_ma(MA_X)
                self.control.setXdelta(x_delta_MA)
                velocity_x_command = self.control.getMovementVelocityXCommand()
                # yaw_command = self.control.getMovementYawAngle()

                self.control.control_drone()
                # draw lidar distance
                self.prepare_visualisation(obj_dist, obj_center, obj_to_track, color, velocity_x_command, x_delta, y_delta, velocity_z_command)
            else:
                return "search"



# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # Args parser
    parser = argparse.ArgumentParser(description='Drive autonomous')
    parser.add_argument('--debug_path', type=str, default="debug/run1", help='debug message name')
    parser.add_argument('--mode', type=str, default='test',
                        help='Switches between flight record and flight visualisation')
    parser.add_argument('--control', type=str, default='PID', help='Use PID or P controller')
    args = parser.parse_args()

    mainflow = Mainflow()
    mainflow.run()

