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
    #cmd_topic.publish(msg)
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

print("finished")

