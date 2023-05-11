import sys, time
from drone import droneLib

#config 
height = 10
speed = 3 #m/s
size = 165 #10 meter  
#end config

droneLib = droneLib()
droneLib.connect_drone('127.0.0.1:14551')
# drone.connect_drone('/dev/ttyACM0')
# drone.connect_drone('127.0.0.1:14551')

# droneLib.arm_and_takeoff(height)

time.sleep(5)

#fly recangle with XYZ
for i in range(size):
    droneLib.send_movement_command_XYZ(speed,0,0)
    time.sleep(0.02)

time.sleep(5)

for i in range(size):
    droneLib.send_movement_command_XYZ(0,speed,0)
    time.sleep(0.02)

time.sleep(5)

for i in range(size):
    droneLib.send_movement_command_XYZ(-speed,0,0)
    time.sleep(0.02)

time.sleep(5)

for i in range(size):
    droneLib.send_movement_command_XYZ(0,-speed,0)
    time.sleep(0.02)

time.sleep(5)

#fly rectangle with YAW

for i in range(4):
    if i > 0:
        droneLib.send_movement_command_YAW(90)

    for i in range(size):
        droneLib.send_movement_command_XYZ(speed,0,0)
        time.sleep(0.02)

    time.sleep(5)

droneLib.land()