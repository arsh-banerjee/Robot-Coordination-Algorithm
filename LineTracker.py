import cv2
import roslibpy
import time
import numpy as np

# define a video capture object
vid = cv2.VideoCapture(4)
client = roslibpy.Ros(host='192.168.1.180', port=9090)
client.run()
cmd_topic = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist', throttle_rate=50)
cmd_topic.advertise()

angular_speed = 0
linear_speed = 0.05

left_pixel = (300,50)
right_pixel = (300,590)

while (True):
    # Capture the video frame
    # by frame
    ret, frame = vid.read()
    center_pixel = (round(frame.shape[0] / 2), round(frame.shape[1] / 2))
    center_pixel_rgb = frame[center_pixel[0],center_pixel[1],:]
    left_pixel_rgb = frame[left_pixel[0],left_pixel[1],:]
    right_pixel_rgb = frame[right_pixel[0], right_pixel[1], :]

    if np.min(right_pixel_rgb) > 150:
        while(np.min(right_pixel_rgb) > 150):
            angular_speed -= 0.01
            msg = roslibpy.Message({'linear': {'x': linear_speed, 'y': 0.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': angular_speed}})
            cmd_topic.publish(msg)
            ret, frame = vid.read()
            right_pixel_rgb = frame[right_pixel[0], right_pixel[1], :]
        angular_speed = 0
        msg = roslibpy.Message({'linear': {'x': linear_speed, 'y': 0.0, 'z': 0.0},
                                'angular': {'x': 0.0, 'y': 0.0, 'z': angular_speed}})
        cmd_topic.publish(msg)

    if np.min(left_pixel_rgb) > 150:
        while(np.min(left_pixel_rgb) > 150):
            angular_speed += 0.01
            msg = roslibpy.Message({'linear': {'x': linear_speed, 'y': 0.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': angular_speed}})
            cmd_topic.publish(msg)
            ret, frame = vid.read()
            left_pixel_rgb = frame[left_pixel[0], left_pixel[1], :]
        angular_speed = 0
        msg = roslibpy.Message({'linear': {'x': linear_speed, 'y': 0.0, 'z': 0.0},
                                'angular': {'x': 0.0, 'y': 0.0, 'z': angular_speed}})
        cmd_topic.publish(msg)

    msg = roslibpy.Message({ 'linear': {'x': linear_speed, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': angular_speed}})

    cmd_topic.publish(msg)

    # Display the resulting frame
    cv2.imshow('frame', frame)

    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
client.terminate()






