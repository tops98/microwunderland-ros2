class ArucuDetector:
    _detector:aruco.ArucoDetector

    def __init__(self, dictionary_type:str) -> None:
        if dictionary_type not in  vars(aruco).keys():
            raise ValueError("the given dictionary is unknown")
        
        aruco_dict = aruco.getPredefinedDictionary(vars(aruco)[dictionary_type])
        parameters =  aruco.DetectorParameters()
        self._detector = aruco.ArucoDetector(aruco_dict,parameters)

    def detect_markers(self, image:np.ndarray):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        (corners,ids,rejected) =  self._detector.detectMarkers(gray)
        return (corners,ids,rejected) 
    
    def convert_to_Named2DPositions(self, image:np.ndarray, markers) -> List[Named2DPosition]:
        (corners,ids,rejected) = markers
        valid_detections = list()
        
        if len(corners) > 0:
            ids = ids.flatten()

            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                new_pos = Named2DPosition()
                new_pos.x = int((topLeft[0] + bottomRight[0]) / 2.0)
                new_pos.y = int((topLeft[1] + bottomRight[1]) / 2.0)
                new_pos.name = str(markerID)
                valid_detections.append(new_pos)

        return valid_detections
    
    def draw_predictions(self, image:np.ndarray, markers) -> np.ndarray:
        (corners,ids,rejected) = markers
        frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
        return frame_markers