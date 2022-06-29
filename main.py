import pyzed.sl as sl
import cv2

init = sl.InitParameters()
init.camera_resolution = sl.RESOLUTION.HD720
init.depth_mode = sl.DEPTH_MODE.PERFORMANCE


ip = "192.168.1.180"
init.set_from_stream(ip)

cam = sl.Camera()
status = cam.open(init)
if status != sl.ERROR_CODE.SUCCESS:
    print(repr(status))
    exit(1)


runtime = sl.RuntimeParameters()
mat = sl.Mat()

key = ''
print("  Quit : CTRL+C\n")
while key != 113:
    err = cam.grab(runtime)
    if (err == sl.ERROR_CODE.SUCCESS) :
        cam.retrieve_image(mat, sl.VIEW.LEFT)
        cv2.imshow("ZED", mat.get_data())
        key = cv2.waitKey(1)
    else :
        key = cv2.waitKey(1)
        cam.close()
