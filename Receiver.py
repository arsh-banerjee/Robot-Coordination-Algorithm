import sys
import numpy as np
import pyzed.sl as sl
import cv2
import time
import roslibpy

#client = roslibpy.Ros(host='192.168.1.190', port=9090)
#client.run()
#cmd_topic = roslibpy.Topic(client, '/joy_vel', 'geometry_msgs/Twist', throttle_rate=50)
#cmd_topic.advertise()


def pathPrediction(xVel, yVel, intPos):
    avg_time = 10
    futurePos = [None] * avg_time
    futurePos[0] = intPos
    xVel = xVel * 1000
    yVel = yVel * 1000

    for i in range(avg_time - 1):
        xPos = intPos[0] + xVel * (i + 1)
        yPos = intPos[1] + yVel * (i + 1)
        futurePos[i + 1] = (xPos, yPos)

    return futurePos


def CollisionDetection(Paths, k, col_Dist):
    if k == 1: return
    Paths = np.array(Paths)
    for i in range(len(Paths[0])):
        time_slice = (Paths[:, i])
        dist = np.linalg.norm(time_slice - time_slice[:, None], axis=-1)
        minDist = np.min(dist[np.nonzero(dist)])
        if minDist < col_Dist:
            return True, np.where(dist == minDist), minDist, i
    return False, np.where(dist == minDist), minDist, i


def crashCorrection(Paths, Objects, minDist, thres_dist, i):
    vel_change = (thres_dist-minDist)/(1.5*1000)
    if vel_change < 0: vel_change = 0
    Paths = np.array(Paths)
    PathsOI = Paths[Objects[0][0]:Objects[0][1]+1, :]
    speed1 = np.linalg.norm(PathsOI[0][0]-PathsOI[0][2])/(2*1000)
    speed2 = np.linalg.norm(PathsOI[1][0]-PathsOI[1][2])/(2*1000)
    if speed1>speed2:
        msg = roslibpy.Message({
            'linear': {'x': round(speed1 - vel_change, 2), 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}})
    else:
        msg = roslibpy.Message({
            'linear': {'x': round(speed2 - vel_change, 2), 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}})
    cmd_topic.publish(msg)
    return round(vel_change,2)

path = "./"

init = sl.InitParameters()
init.camera_resolution = sl.RESOLUTION.HD720
init.depth_mode = sl.DEPTH_MODE.PERFORMANCE


ip = '192.168.10.3'
init.set_from_stream(ip)

zed = sl.Camera()
status = zed.open(init)
if status != sl.ERROR_CODE.SUCCESS:
    print(repr(status))
    exit(1)

runtime = sl.RuntimeParameters()

key = ' '

Tracker_dict = {'csrt': cv2.TrackerCSRT_create, 'kcf': cv2.TrackerKCF_create, 'boosting': cv2.TrackerBoosting_create,
                'mil': cv2.TrackerMIL_create, 'medianflow': cv2.TrackerMedianFlow_create, 'mosse': cv2.TrackerMOSSE_create}

trackers = cv2.MultiTracker_create()

# Prepare new image size to retrieve half-resolution images
image_size = zed.get_camera_information().camera_resolution
image_size.width = image_size.width
image_size.height = image_size.height

# Declare your sl.Mat matrices
image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
depth_image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
point_cloud = sl.Mat()

# reading frames from the video
err = zed.grab(runtime)
zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
frame = cv2.cvtColor(image_zed.get_data(), cv2.COLOR_BGRA2BGR)

# Number of objects to track
k = 2

boundingboxes = [(223, 394, 166, 129),(746, 416, 250, 151)]

for i in range(k):
    boundingbox = boundingboxes[i]
    tracker = Tracker_dict['csrt']()
    trackers.add(tracker, frame, boundingbox)

col_timer = time.time()
collision_once = False

path_history = [[] for i in range(k)]
current_speeds = [[0, 0] for i in range(k)]
start = time.time()
compTime = 1  # Calculate every x seconds
rollingTime = 2
Paths = [None] * k
collision_distance = 200

while True:
    err = zed.grab(runtime)
    if err == sl.ERROR_CODE.SUCCESS:
        # Retrieve the left image, depth image in the half-resolution
        zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)

        # Retrieve the RGBA point cloud in half resolution
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, image_size)

        # To recover data from sl.Mat to use it with opencv, use the get_data() method
        image_ocv = image_zed.get_data()

        frame = cv2.cvtColor(image_zed.get_data(), cv2.COLOR_BGRA2BGR)
        (success, boxes) = trackers.update(frame)

        if time.time() - start > compTime:
            for i in range(len(boxes)):  # Save current location to path history for bounding boxes every time step
                (x, y, w, h) = [int(a) for a in boxes[i]]
                err, point = point_cloud.get_value(int(x + 0.5 * w), int(y + h * 0.5))
                path_history[i].append(point)

            if len(path_history[0]) > (rollingTime / compTime):
                for i in range(len(boxes)):
                    index = int(len(path_history[0]) - 1)
                    index2 = int(len(path_history[0]) - (rollingTime / compTime) - 1)
                    p1 = path_history[i][index]
                    p2 = path_history[i][index2]
                    xDistance = (p1[0] - p2[0])
                    xSpeed = round((xDistance / rollingTime) / 1000, 2)
                    yDistance = (p1[1] - p2[1])
                    ySpeed = round((yDistance / rollingTime) / 1000, 2)
                    Paths[i] = pathPrediction(xSpeed, ySpeed, (p1[0], p1[1]))
                    current_speeds[i] = [xSpeed, ySpeed]
                    # print("\rObject " + str(i+1) + " Speed: " + str((xSpeed,ySpeed)) + " m/s",end='')
                Crash, Objects, minDist, i = CollisionDetection(Paths, k, collision_distance)
                if Crash:
                    collision_once = True
                    vel_cor = crashCorrection(Paths, Objects, minDist, collision_distance, i)
                    print("\rCollision Occurance: " + str(Crash) + " , occurs between objects " + str(Objects[0] + 1)
                          + " Correction: " + str(vel_cor) + " m/s" , end='')
                else:
                    print("\rCollision Occurance: " + str(Crash), end='')
            start = time.time()

client.terminate()

zed.close()
