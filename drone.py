from dronekit import *


class droneLib:
    def __init__(self):
        self.vehicle = None

    # Connect to the Vehicle (in this case a UDP endpoint)
    def connect_drone(self,connection_string, waitready=True, baudrate=57600):
        if self.vehicle == None:
            self.vehicle = connect(connection_string, wait_ready=waitready, baud=baudrate)
        print("drone connected")

    def disconnect_drone(self):
        self.vehicle.close()

    def get_version(self):
        return self.vehicle.version

    def get_mission(self):
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        return

    def get_location(self):
        return self.vehicle.location.global_frame

    def get_altitude(self):
        return self.vehicle.location.global_relative_frame.alt

    def get_velocity(self):
        return self.vehicle.velocity

    def get_battery_info(self):
        global vehicle
        return self.vehicle.battery

    def get_mode(self):
        return self.vehicle.mode.name

    def get_home_location(self):
        return self.vehicle.home_location

    def get_heading(self):
        return self.vehicle.heading

    def get_EKF_status(self):
        return self.vehicle.ekf_ok

    def get_ground_speed(self):
        return self.vehicle.groundspeed

    def read_channel(self,channel):
        return self.vehicle.channels[str(channel)]

    def set_gimbal_angle(self,angle):
        print("gimbal angle set to: " % angle)
        return self.vehicle.gimbal.rotate(0, angle, 0)

    def set_groundspeed(self,speed):
        print("groundspeed set to: " % speed)
        self.vehicle.groundspeed = speed

    def set_flight_mode(self,f_mode):
        self.vehicle.mode = VehicleMode(f_mode)

    def set_param(self,param, value):
        global vehicle
        self.vehicle.parameters[param] = value

    def get_param(self,param):
        return self.vehicle.parameters[param]

    def set_channel(self,channel, value):
        self.vehicle.channels.overrides[channel] = value

    def clear_channel(self,channel):
        self.vehicle.channels.overrides[channel] = None

    def get_channel_override(self,channel):
        return self.vehicle.channels.overrides[channel]

    def disarm(self):
        self.vehicle.armed = False

    def arm(self):
        self.vehicle.groundspeed = 3

        print ("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print (" Waiting for vehicle to initialise...")
            time.sleep(1)

        print ("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode    = VehicleMode("STABILIZE")
        self.vehicle.armed   = True

        while not self.vehicle.armed:
            print (" Waiting for arming...")
            time.sleep(1)

        print ("ARMED! Let's take OFF")

    def arm_and_takeoff(self,aTargetAltitude):
        #set default groundspeed low for safety
        print ("setting groundspeed to 3")
        self.vehicle.groundspeed = 3

        print ("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print (" Waiting for vehicle to initialise...")
            time.sleep(1)

        print ("Arming motors")
        # Copter should arm in GUIDED mode#################
        self.vehicle.armed   = True
        self.vehicle.mode    = VehicleMode("GUIDED")



        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print (" Waiting for arming...")
            time.sleep(1)

        print ("Taking off!")
        #simple_takeoff
        self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print (" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            #Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
                print ("Reached target altitude")
                break
            time.sleep(1)

    def land(self):
        print("Setting LAND mode...")
        self.vehicle.mode = VehicleMode("LAND")

    def return_to_launch_location(self):
        #carefull with using this! It wont detect obstacles!
        self.ehicle.mode = VehicleMode("RTL")

    def send_movement_command_YAW(self,heading):
        speed = 0
        direction = 1 #direction -1 ccw, 1 cw

        #heading 0 to 360 degree. if negative then ccw

        print("Sending YAW movement command with heading: %f" % heading)

        if heading < 0:
            heading = heading*-1
            direction = -1

        #point drone into correct heading
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading,
            speed,      #speed deg/s
            direction,
            1,          #relative offset 1
            0, 0, 0)

        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        #Vehicle.commands.flush()

    def send_movement_command_XYZ(self,velocity_x, velocity_y, velocity_z,duration=1):   #,altitude

        #velocity_x positive = forward. negative = backwards
        #velocity_y positive = right. negative = left
        #velocity_z positive = down. negative = up (Yes really!)

        print("Sending XYZ movement command with v_x(forward/backward): %f v_y(right/left): %f " % (velocity_x,velocity_y))

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,  #relative to drone heading pos relative to EKF origin
            0b0000111111000111, #ignore velocity z and other pos arguments
            0, 0, 0,
            velocity_x, velocity_y, velocity_z,
            0, 0, 0,
            0, 0)

        for x in range(0, duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)

