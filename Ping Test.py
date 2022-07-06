import pyzed.sl as sl
import cv2

# Set the input from stream
init = sl.InitParameters()
init.set_from_stream("173.54.189.63", 30000) # Specify the IP and port of the sender

zed = sl.Camera()
# Open the camera
err = zed.open(init)
if err != sl.ERROR_CODE.SUCCESS:
	exit(1)

while True:
    if (zed.grab() == sl.ERROR_CODE.SUCCESS):
        print("works")

# Close the camera
zed.close()