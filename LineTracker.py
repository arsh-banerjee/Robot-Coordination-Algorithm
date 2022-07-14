import cv2
import roslibpy
import time
# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
rawCapture = PiRGBArray(camera)
# allow the camera to warmup
time.sleep(0.1)
# grab an image from the camera

client = roslibpy.Ros(host='192.168.1.180', port=9090)
client.run()
cmd_topic = roslibpy.Topic(client, '/joy_vel_0', 'geometry_msgs/Twist', throttle_rate=50)
cmd_topic.advertise()

while True:
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array
    print(image)
    if False:
        linear_speed = 0.05
        angular_speed = 0

    msg = roslibpy.Message({
        'linear': {'x': linear_speed, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    })

    cmd_topic.publish(msg)

    client.terminate()
