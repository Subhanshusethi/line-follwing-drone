import numpy as np
import cv2
import math
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
cap = cv2.VideoCapture(2)
labVals = [0,153,0,255,255,255]



sensors = 3
threshold = 0.2
width, height = 480, 360
senstivity = 3  # if number is high less sensitive
weights = [-25, -15, 0, 15, 25]
fSpeed = 1
curve = 0
prev_cx, prev_cy = None, None
prev_biggest = None
start_time = None
max_contour_time = 1
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
kp=0.032 #1564
kd=0.008#656
ki=0
kernel = np.ones((2, 2), np.uint8)
kernel1 = np.ones((55, 55), np.uint8)
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
    while True:
        print (" Altitude: "), vehicle.location.global_relative_frame.alt
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)
arm_and_takeoff(12)
def thresholding(img):
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    lower = np.array([labVals[0], labVals[1], labVals[2]])
    upper = np.array([labVals[3], labVals[4], labVals[5]])
    mask = cv2.inRange(lab, lower, upper)
    return mask
def getContours(imgThres, img):

    cx = 0
    cy = 0
    contours, hierarchy = cv2.findContours(imgThres, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) != 0:
        biggest = max(contours, key=cv2.contourArea)

        M = cv2.moments(biggest)
        
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        cv2.drawContours(img, biggest, -1, (255, 0, 255), 7)
        cv2.circle(img, (cx, cy), 10, (0, 255, 0), cv2.FILLED)
    return cx, cy ,M

def calculate_velocity(angle):
    velocity_x = fSpeed * math.cos(angle)
    velocity_y = fSpeed * math.sin(angle)
    return velocity_x, velocity_y

def calculate_angle(dx, dy):
    return math.atan2(dy, dx)
def getSensorOutput(imgThres, sensors):
    imgs = np.hsplit(imgThres, sensors)
    totalPixels = (imgThres.shape[1] // sensors) * imgThres.shape[0]
    senOut = []
    for x, im in enumerate(imgs):
        pixelCount = cv2.countNonZero(im)
        if np.any(pixelCount > threshold * totalPixels):
            senOut.append(1)
        else:
            senOut.append(0)
     #   cv2.imshow(str(x), im)

    print(senOut)

    return senOut
def on_kp_change(value):
    global kp
    kp = value/1000


def on_ki_change(value):
    global ki
    ki = value/1000


def on_kd_change(value):
    global kd
    kd = value/1000
def send_velocity(velocity_x,velocity_y):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
                0,       # time_boot_ms (not used)
                0, 0,    # target system, target component
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
                0b0000111111000111,  # type_mask (only speeds enabled)
                0, 0, 0,  # x, y, z positions (not used)
                velocity_x,  # velocity_x in m/s
                velocity_y,  # velocity_y in m/s
                0,  # velocity_z in m/s (not used)
                0, 0, 0,  # accelerations (not supported yet, ignored in GCS_Mavlink)
                0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    vehicle.send_mavlink(msg)

roll=0
pitch=0
yaw=0

rollkp = 0
rollkd = 0
rollki = 0

while True:
    _, img = cap.read()
    zoom_factor = 2.3
    img = cv2.resize(img, (width, height))
    new_height = int(height * zoom_factor)
    new_width = int(width * zoom_factor)
    zoomed_image = cv2.resize(img, (new_width, new_height))
    imgThres = thresholding(zoomed_image)
    eroded_image = cv2.erode(imgThres, kernel, iterations=1)
    dilated_image = cv2.dilate(eroded_image, kernel1, iterations=1)


    SenOut = getSensorOutput(dilated_image, sensors)
    cx = 0
    cy = 0
    contours, hierarchy = cv2.findContours(dilated_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) != 0:
        biggest = max(contours, key=cv2.contourArea)
        M = cv2.moments(biggest)
        if M['m00'] != 0:  # Check if denominator is not zero
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        cv2.drawContours(zoomed_image, biggest, -1, (255, 0, 255), 7)
        cv2.circle(zoomed_image, (cx, cy), 10, (0, 255, 0), cv2.FILLED)

     

        if prev_cx is not None and prev_cy is not None:
            if M['m00']!=0:

                dx = cx - prev_cx
                dy = cy - prev_cy

                rollkp=kp*cx
                rollkd = rollkd - kd * (cx - prev_cx)
                rollki = rollki - ki * (cx + prev_cx)
                tuneerror=rollkp+rollkd+rollki
                angle = calculate_angle(tuneerror, dy)
                velocity_x, velocity_y = calculate_velocity(angle)
                print("Angle:", math.degrees(angle))
                print("Velocity X:", velocity_x)
                print("Velocity Y:", velocity_y)
                print("tuneerror",tuneerror)
                print(cx,cy)
            else:
                cx = 0
                cy = 0
                velocity_x = 0
                velocity_y = 0


            send_velocity(velocity_x,velocity_y)
            time.sleep(0.05)
            roll = vehicle.attitude.roll
            pitch = vehicle.attitude.pitch
            yaw = vehicle.attitude.yaw
            print("roll", roll)
            print("pitch", pitch)
            print("yaw", yaw)
            data = {
                'velocity_x': velocity_x,
                'velocity_y': velocity_y,
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw
            }
            velocity_x_value = data['velocity_x']
            velocity_y_value = data['velocity_y']
            roll_value = data['roll']
            pitch_value = data['pitch']
            yaw_value = data['yaw']
            print("rollangle", roll_value)
            print("pitchangle", pitch_value)
            print("yawangle", yaw_value)
        prev_cx = cx
        prev_cy = cy
        cv2.imshow('ORIGINAL', zoomed_image)
        cv2.imshow("Dilated Image", dilated_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
cv2.destroyAllWindows()
cap.release()
print("Landing...")
vehicle.mode = VehicleMode("LAND")

vehicle.close()


