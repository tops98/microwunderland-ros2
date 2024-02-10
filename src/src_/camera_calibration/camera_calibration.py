import numpy as np
import cv2 as cv
import glob


# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

num_rows = 10
num_cols = 6


# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((num_rows*num_cols,3), np.float32)
objp[:,:2] = np.mgrid[0:num_cols,0:num_rows].T.reshape(-1,2)


# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# prepare camera 
cam_port = 0
cam = cv.VideoCapture(cam_port,cv.CAP_DSHOW)

capture_width = 640
capture_height = 480

# set camera resolution
cam.set(cv.CAP_PROP_FRAME_WIDTH, capture_width)
cam.set(cv.CAP_PROP_FRAME_HEIGHT, capture_height)

actual_w = int(cam.get(cv.CAP_PROP_FRAME_WIDTH))
actual_h = int(cam.get(cv.CAP_PROP_FRAME_HEIGHT))

if actual_w != capture_width or actual_h != capture_height:
    print(f"""
the requested resolution is not available:
    => suggested resolution is: {actual_w}, {actual_h}
""")
    quit()



# set number of required successful scans
num_scans = 10 

print("STARTING CAPTURE PHASE:\n\r press 'c' to freeze the current frame and initiate a scan")
while len(objpoints) < num_scans:
    # capture frame and show it
    res,img = cam.read()   
    cv.imshow("preview", img)

    if cv.waitKey(30) == ord('c'):

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (num_cols,num_rows), None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            print("scann successful")
            objpoints.append(objp)

            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
            
            # Draw and display the corners
            cv.drawChessboardCorners(img, (num_cols,num_rows), corners2, ret)
            cv.imshow('preview', img)
        else:
            print("scann failed")
        
        print("press any key to continue")
        cv.waitKey(0)

cv.destroyAllWindows()

print("""
Required number of scans captured !
calculating calibration parameters...
""")

# calculate camera and distortion matrix, rotation and translation vectors
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# get optimized matrix
optimized_mtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (capture_width,capture_height), 1, (capture_width,capture_height))

# get undistort map
mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, optimized_mtx, (capture_width,capture_height), 5)



# test calibration parameter on new image
print("""
capture image to test calibration parameters
=> press 'c' to capture an image
""")

while cv.waitKey(30) != ord('c'):
    # capture frame and show it
    res,img = cam.read()   
    cv.imshow("preview", img)

print("press any key to close and save the camera calibration data")
dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)


cv.imshow("undistorted", dst)
cv.imshow("distorted", img)
cv.waitKey(0)
np.savetxt('camera_calibration_data.txt', (mapx,mapy), delimiter=',')
print("calibration data saved")