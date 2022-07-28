import cv2
import pyzed.sl as sl
import roslibpy
import base64
import logging
import time

client = roslibpy.Ros(host='192.168.1.206', port=9090)
client.run()
publisher = roslibpy.Topic(client, '/camera/image/compressed', 'sensor_msgs/CompressedImage')


# Create a ZED camera object
zed = sl.Camera()

# Camera Initilization (Pull feed from stream)
init = sl.InitParameters()
init.camera_resolution = sl.RESOLUTION.HD720
init.depth_mode = sl.DEPTH_MODE.PERFORMANCE


zed = sl.Camera()
err = zed.open(init)

if err != sl.ERROR_CODE.SUCCESS:
    print(repr(err))
    zed.close()
    exit(1)

# Set runtime parameters after opening the camera
runtime = sl.RuntimeParameters()
runtime.sensing_mode = sl.SENSING_MODE.STANDARD

# Prepare new image size to retrieve half-resolution images
image_size = zed.get_camera_information().camera_resolution
image_size.width = image_size.width/2.5
image_size.height = image_size.height/2.5

# Declare your sl.Mat matrices
image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
depth_image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
point_cloud = sl.Mat()

tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'CSRT', 'MOSSE']
tracker_type = tracker_types[6]


tracker = cv2.legacy.TrackerMOSSE_create()

# reading frames from the video
err = zed.grab(runtime)
zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
frame = cv2.cvtColor(image_zed.get_data(), cv2.COLOR_BGRA2BGR)

# Uncomment the line below to select a different bounding box
bbox = cv2.selectROI(frame, False)

# Initialize tracker with first frame and bounding box
ok = tracker.init(frame, bbox)

p1_past = [0,0]


bbox_initial = bbox
start = time.time()

fps = 0


timer = time.time()

while True:
    err = zed.grab(runtime)
    zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
    image_ocv = image_zed.get_data()
    frame = cv2.cvtColor(image_zed.get_data(), cv2.COLOR_BGRA2BGR)

    # Update tracker
    ok, bbox = tracker.update(frame)

    # Calculate Frames per second (FPS)
    fps += 1

    # Draw bounding box
    if ok:
        # Tracking success
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
    else :
        # Tracking failure
        cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

    # Display tracker type on frame
    cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);

    # Display FPS on frame
    cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);

    # Display result
    cv2.imshow("Tracking", frame)

    framerate = 25
    if time.time() - timer > 1/framerate:
        cv2.imwrite("Frame.jpg", frame)
        image_bytes = open('Frame.jpg', 'rb').read()
        encoded = base64.b64encode(image_bytes).decode('ascii')
        publisher.publish(dict(format='jpeg', data=encoded))
        timer = time.time()
    # Exit if ESC pressed
    k = cv2.waitKey(1) & 0xff
    if k == 27 : break
