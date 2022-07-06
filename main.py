import pyzed.sl as sl
import cv2

init_params = sl.InitParameters() # Set initial parameters
init_params.sdk_verbose = 1 # Enable verbose mode
input_t = sl.InputType()
input_t.set_from_stream("173.54.189.63")
init_params.input = input_t
init_params.set_from_stream("173.54.189.63") # You can also use this

cam = sl.Camera()
status = cam.open(init_params)
#if status != sl.ERROR_CODE.SUCCESS:
    #print(repr(status))
    #exit(1)


runtime = sl.RuntimeParameters()
mat = sl.Mat()

key = ''
print("  Quit : CTRL+C\n")
while key != 113:
    err = cam.grab(runtime)
    if (err == sl.ERROR_CODE.SUCCESS) :
        cam.retrieve_image(mat, sl.VIEW.LEFT)
        print(mat)
        key = cv2.waitKey(1)
    else :
        key = cv2.waitKey(1)
        cam.close()
