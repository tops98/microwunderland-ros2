import cv2
from cv2 import aruco

frame = cv2.imread("/home/tobias/micro-wonderland_unity/Assets/Exports/Labeled_Images/car_mix/9.jpg")
frame = cv2.imread("/home/tobias/Desktop/Untitled.jpg")

# aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
# aruco_params = aruco.DetectorParameters()
# aruco_detector = aruco.ArucoDetector()
# corners,ids,rejected = aruco_detector.detectMarkers(frame)
# print((corners,ids,rejected))
# frame = aruco.drawDetectedMarkers(frame,rejected,ids,(0,255,0))

def detect(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
    parameters =  aruco.DetectorParameters()
    corners, ids, rejectedImgPoints = aruco.ArucoDetector(aruco_dict,detectorParams=parameters).detectMarkers(gray)
    frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
    return frame_markers

# a = detect(cv2.imread("/home/tobias/Desktop/Untitled.jpg"))
b = detect(cv2.imread("/home/tobias/micro-wonderland_unity/Assets/Exports/Labeled_Images/car_mix/9.jpg"))
# c = detect(cv2.imread("/home/tobias/Desktop/test_aruco.png"))

# cv2.imshow("asda",a)
cv2.imshow("rgrgb",b)
# cv2.imshow("rgrgb",c)

cv2.waitKey(0)
cv2.destroyAllWindows()