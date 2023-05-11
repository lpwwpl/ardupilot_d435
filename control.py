from logging import debug
from drone import droneLib
from simple_pid import PID
import time

USE_PID_YAW = True
USE_PID_ROLL = False

MAX_SPEED = 4       # M / s
MAX_YAW = 15        # Degrees / s 

P_YAW = 0.02 #orgineel 0.01
I_YAW = 0
D_YAW = 0

P_ROLL = 0.22
I_ROLL = 0
D_ROLL = 0

P_PITCH = 0.02
I_PITCH = 0
D_PITCH = 0


class ArdupilotController:
    def __init__(self):
        self.droneLib = droneLib()
        self.control_loop_active = True
        self.pidYaw = None
        self.pidRoll = None
        self.pidPitch = None

        self.movementYawAngle = 0
        self.movementRollAngle = 0
        self.movementPitchAngle = 0

        self.inputValueYaw = 0
        self.inputValueVelocityY = 0
        self.inputValueVelocityZ = 0
        self.inputValueVelocityX = 0

        self.control_loop_active = True
        self.flight_altitude = 4

        self.debug_yaw = None
        self.debug_velocity = None

    def configure_PID(self,control):

        """ Creates a new PID object depending on whether or not the PID or P is used """
        print("Configuring control")

        if control == 'PID':
            self.pidYaw = PID(P_YAW, I_YAW, D_YAW, setpoint=0)       # I = 0.001
            # self.pidYaw.output_limits = (-MAX_YAW, MAX_YAW)
            self.pidYaw.output_limits = (-MAX_SPEED, MAX_SPEED)          # PID Range
            self.pidRoll = PID(P_ROLL, I_ROLL, D_ROLL, setpoint=0)   # I = 0.001
            self.pidRoll.output_limits = (-MAX_SPEED, MAX_SPEED)     # PID Range

            self.pidPitch = PID(P_PITCH,I_PITCH,D_PITCH,setpoint=0)
            self.pidPitch.output_limits = (-MAX_SPEED,MAX_SPEED)
            print("Configuring PID")
        else:
            self.pidYaw = PID(P_YAW, 0, 0, setpoint=0)               # I = 0.001
            # self.pidYaw.output_limits = (-MAX_YAW, MAX_YAW)          # PID Range
            self.pidYaw.output_limits = (-MAX_SPEED, MAX_SPEED)
            self.pidRoll = PID(P_ROLL, 0, 0, setpoint=0)             # I = 0.001
            self.pidRoll.output_limits = (-MAX_SPEED, MAX_SPEED)     # PID Range
            self.pidPitch = PID(P_ROLL, 0, 0, setpoint=0)             # I = 0.001
            self.pidPitch.output_limits = (-MAX_SPEED, MAX_SPEED)     # PID Range

            print("Configuring P")

    def connect_drone(self,drone_location):
        self.droneLib.connect_drone(drone_location) #'/dev/ttyACM0'

    def getMovementYawAngle(self):
        return self.movementYawAngle

    def setYdelta(self,YDelta):
        self.inputValueVelocityZ = YDelta

    def setXdelta(self,XDelta):
        # self.inputValueYaw = XDelta
        self.inputValueVelocityY = XDelta

    def getMovementVelocityXCommand(self):
        return self.movementRollAngle

    def getMovementVelocityYCommand(self):
        return self.movementYawAngle

    def setZDelta(self,ZDelta):
        self.inputValueVelocityX = ZDelta

    def set_system_state(self,current_state):
        self.state = current_state

    def set_flight_altitude(self,alt):
        self.flight_altitude = alt
    # end control functions

    #drone functions
    def arm_and_takeoff(self,max_height):
        self.droneLib.arm_and_takeoff(max_height)

    def land(self):
        self.droneLib.land()

    def print_drone_report(self):
        print(self.droneLib.get_EKF_status())
        print(self.droneLib.get_battery_info())
        print(self.droneLib.get_version())
    #end drone functions

    def initialize_debug_logs(self,DEBUG_FILEPATH):
        global debug_yaw, debug_velocity
        self.debug_yaw = open(DEBUG_FILEPATH + "_yaw.txt", "a")
        self.debug_yaw.write("P: I: D: Error: command:\n")

        self.debug_velocity = open(DEBUG_FILEPATH + "_velocity.txt", "a")
        self.debug_velocity.write("P: I: D: Error: command:\n")

    def debug_writer_YAW(self,value):
        self.debug_yaw.write(str(0) + "," + str(0) + "," + str(0) + "," + str(self.inputValueYaw) + "," + str(value) + "\n")

    def debug_writer_ROLL(self,value):
        self.debug_velocity.write(str(0) + "," + str(0) + "," + str(0) + "," + str(self.inputValueYaw) + "," + str(value) + "\n")

    def control_drone(self):
        # if self.inputValueYaw == 0:
        #     self.droneLib.send_movement_command_YAW(0)
        # else:
        #     self.movementYawAngle = (self.pidYaw(self.inputValueYaw) * -1)
        #     self.droneLib.send_movement_command_YAW(self.movementYawAngle)
        #     self.debug_writer_YAW(self.movementYawAngle)
        # and self.inputValueVelocityY == 0 and self.inputValueVelocityZ == 0
        if self.inputValueVelocityY == 0:
            self.droneLib.send_movement_command_XYZ(0,0,0)
        else:
            self.movementYawAngle = (self.pidYaw(self.inputValueVelocityY) * -1)
            print(self.movementRollAngle,self.movementYawAngle,self.movementPitchAngle)
            self.droneLib.send_movement_command_XYZ(0, self.movementYawAngle, 0)
            # self.droneLib.send_movement_command_XYZ(self.movementRollAngle, 0, 0)
            # self.debug_writer_YAW(self.movementYawAngle)
            # self.debug_writer_ROLL(self.movementRollAngle)


        if self.inputValueVelocityX == 0:
            self.droneLib.send_movement_command_XYZ(0, 0, 0)   #,self.flight_altitude
        else:
            self.movementRollAngle = (self.pidRoll(self.inputValueVelocityX) * -1)
            self.droneLib.send_movement_command_XYZ(self.movementRollAngle, 0,0) #self.flight_altitude
            self.debug_writer_ROLL(self.movementRollAngle)





    def stop_drone(self):
        self.droneLib.send_movement_command_YAW(0)
        self.droneLib.send_movement_command_XYZ(0, 0, 0) #self.flight_altitude
    
