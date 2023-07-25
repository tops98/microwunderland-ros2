import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy

from sensor_msgs.msg import CompressedImage
from microwunderland_interfaces.msg import Named2DPosition
from microwunderland_interfaces.msg import BoundingBox
from microwunderland_interfaces.msg import Vector2D

import cv2
from cv2 import aruco
from cv_bridge import CvBridge
import numpy as np
import re
from datetime import datetime
from dataclasses import dataclass
from collections import deque
from  numpy import sin, cos, arccos

import tensorflow as tf
from typing import Dict, List
from microwunderland_interfaces.msg import Named2DPosition
from microwunderland_interfaces.msg import BoundingBox

# Simpel Tracker that assotiates detections, using the 
# eukledian distance, to provide reidentification.
# Additionaly the tracker offers compensation for 
# failed detections of previously identified objects, by 
# extrapolating there position over a set number of samples.
class CentroidTracker:

    @dataclass
    class TrackedObject:
        id:np.uint8
        pos: deque[np.ndarray]
        predicted_pos: np.ndarray
        undetected_cnt:np.uint8

    # list of identified objects that are tracked
    _tracked_objects: Dict[int,TrackedObject]
    # maximum amount of frames before object will be unregistered
    _undetected_max_cnt:np.uint8
    # decay factor for extrapolited headings
    _heading_decay:float
    # maximum distance to asotiate a detection with a given object
    _max_dist:float
    # the next available identification number
    _next_id:np.uint8

    def __init__(self, max_distance:float, undetected_max_cnt:int, heading_decay:float) -> None:
        self._max_dist = max_distance
        self._undetected_max_cnt = undetected_max_cnt
        if heading_decay <= 1 and heading_decay >= 0:
            raise ValueError("heading_decay must be between [0 and 1] (inclusive)")
        self._heading_decay = heading_decay
        self._tracked_objects = dict()

    def update(self, detections:np.ndarray) -> None:
        self._find_knowen_objects(detections)
        self._register_new_objects(detections)
        self._remove_lost_objects()

    # Tries to asociate the new detections with the knowen tracked objects.
    # All reidentified objects will be removed from the detections list
    # and ther positions will be updated according to the newly detected position. 
    # @param detections: list of detections 
    def _find_knowen_objects(self, detections:np.ndarray) -> None:
        for tracked_obj in self._tracked_objects.values():
            closest_detec_id = self._get_closest_detection(tracked_obj,detections)
            if closest_detec_id != -1:

                closest_detec = detections[closest_detec_id]
                np.delete(detections,closest_detec_id)

                tracked_obj.pos.append(np.array[closest_detec.x,closest_detec.y])
                tracked_obj.undetected_cnt = 0
            else:
                tracked_obj.undetected_cnt +=1

            tracked_obj.predicted_pos = self._predict_next_pose(tracked_obj)

    # Returns the index of the detection that is closest to the given
    # tracked object, or -1 if there is no detection within max_dist
    #  
    # @param tracked_object tracked object used as reference point for dist calc
    # @param detections detections to check
    # @returns index of closest detection or -1 if nothing is within max_dist
    def _get_closest_detection(self,tracked_object:TrackedObject ,detections:np.ndarray) -> Named2DPosition:
        closest_detecion_index = -1
        min_dist = self._max_dist

        for index,detection in enumerate(detections):
            dist = np.linalg.norm(tracked_object.pos,detection)
            if dist < min_dist:
                min_dist =dist
                closest_detecion_index = index
                
        return closest_detecion_index

                
    def _register_new_objects(self, detections:List[Named2DPosition]) -> None:
        for detection in detections:
            new_obj = self.TrackedObject()
            new_obj.pos = deque(maxlen=3)
            new_obj.pos.append(np.array[detection.x,detection.y])
            new_obj.pos.append(np.array[detection.x,detection.y])
            new_obj.pos.append(np.array[detection.x,detection.y])
            new_obj.predicted_pos = np.array([detection.x,detection.y])
            new_obj.id = self._next_id
            self._next_id += 1

            self._tracked_objects[new_obj.id] = new_obj
    
    def _remove_lost_objects(self):
        for id,tracked_obj in self._tracked_objects.items():
            if tracked_obj.undetected_cnt > self._undetected_max_cnt:
                self._tracked_objects.pop(id)

    # calculates the angle between two vectors in radients
    def _calculate_angle_with_direction(self, vec_a:np.ndarray, vec_b:np.ndarray):
        unit_vector_1 = vec_a / np.linalg.norm(vec_a)
        unit_vector_2 = vec_b / np.linalg.norm(vec_b)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        angle = np.arccos(dot_product)
        c = np.cross(vec_b,vec_a)
        return angle if c>0 else angle*-1

    # rotates a vector for a given angle in radients
    def _rotate_vector(self, vec:np.ndarray, alpha_rad:float):
        alpha_cos = cos(alpha_rad)
        alpha_sin = sin(alpha_rad)

        rot_matrix = np.array([[alpha_cos,-alpha_sin],[alpha_sin,alpha_cos]])
        rotated_vec = np.dot(rot_matrix,vec)
        return rotated_vec


    def _predict_next_pose(self, positions:deque, heading_decay:float=0.5,undetected_cnt:int=0) -> np.ndarray:
        # calculate motion vectors from last 3 samples
        vec_a = positions[1]-positions[2]
        vec_b = positions[0]-positions[1]
        # calculate angle between motion vectors
        alpha = -1*self._calculate_angle_with_direction(vec_a,vec_b)
        # add accelaration
        accelaration = vec_b-vec_a
        new_vec = vec_b + accelaration
        # apply decay if detections are missing
        new_vec *= heading_decay**undetected_cnt
        # rotate motion vector
        new_pos = positions[0] + self._rotate_vector(new_vec,alpha)
        # calculate next position
        return new_pos



class ArucuDetector:
    _detector:aruco.ArucoDetector

    def __init__(self, dictionary_type:str) -> None:
        if dictionary_type not in  vars(aruco).keys():
            raise ValueError("the given dictionary is unknown")
        
        aruco_dict = aruco.getPredefinedDictionary(vars(aruco)[dictionary_type])
        parameters =  aruco.DetectorParameters()
        self._detector = aruco.ArucoDetector(aruco_dict,parameters)

    def __detect_markers(self, image:np.ndarray):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        (corners,ids,rejected) =  self._detector.detectMarkers(gray)
        return (corners,ids,rejected) 
    
    def predict_positions(self, image:np.ndarray) -> List[Named2DPosition]:
        (corners,ids,rejected) = self.__detect_markers(image)
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
    
    def draw_predictions(self, image:np.ndarray) -> np.ndarray:
        (corners,ids,rejected) = self.__detect_markers(image)
        frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
        return frame_markers
    
    
class ObjectDetector:
    _interpreter:tf.lite.Interpreter
    _input_details:list
    _output_details:list
    _label_map: Dict
    _mean: float
    _std: float

    def __init__(self, model_path:str, label_map_path:str, std:float=127.5, mean:float=127.5) -> None:
        self._mean = mean
        self._std = std
        self._label_map = self.read_label_map(label_map_path)
        self._interpreter = tf.lite.Interpreter(model_path)
        self._interpreter.allocate_tensors()
        self._input_details = self._interpreter.get_input_details()
        self._output_details = self._interpreter.get_output_details()

    def read_label_map(self, label_map_path):

        item_id = None
        item_name = None
        items = {}

        with open(label_map_path, "r") as file:
            pattern = r"(item\s*{[^}]+})"
            matches = re.findall(pattern,file.read())
            for match in matches:
                id = int(re.search(r"(id\s*:\s*)(\d+)",match)[2])
                display_name = re.search(r'(display_name\s*:\s*")(\w+)',match)[2]
                items[id] = display_name
                  

        return items

    def _preprocess_image(self, image:np.ndarray) -> np.ndarray:
        # resize image to input shape of model
        image = cv2.resize(image, self._input_details[0]['shape'][1:3])
        # add axis, since model usally expects batches
        image = np.expand_dims(image,axis=0)
        # standardize data if float input is used
        if self._input_details[0]['dtype'] == np.float32:
            image = (np.float32(image) - self._mean)/self._std
        return image
    
    def _predict(self, image:np.ndarray) -> Dict[str,any]:
        processedImg = self._preprocess_image(image)
        self._interpreter.set_tensor(self._input_details[0]['index'], processedImg)
        self._interpreter.invoke()
        predictions = dict()

        predictions["boxes"] = (self._interpreter.get_tensor(self._output_details[1]['index'])[0])
        predictions["classes"] = (self._interpreter.get_tensor(self._output_details[3]['index'])[0])
        predictions["confidence"] = (self._interpreter.get_tensor(self._output_details[0]['index'])[0])

        return predictions

    def preidict_bounds(self, image:np.ndarray, confidence_limit=0.75) -> List[BoundingBox]:
        pred = self._predict(image)
        valid_detections = list()

        for i,conf in enumerate(pred["confidence"]):
            if conf < confidence_limit:
                break

            new_bb = BoundingBox()
            vec_a = Vector2D()
            vec_b = Vector2D()

            vec_a.x = (image.shape[0]*pred["boxes"][i][1])
            vec_a.y = (image.shape[1]*pred["boxes"][i][0])

            vec_b.x = (image.shape[0]*pred["boxes"][i][3])
            vec_b.y = (image.shape[1]*pred["boxes"][i][2])

            new_bb.vec_a = vec_a
            new_bb.vec_b = vec_b
            new_bb.label_name = self._label_map[int(pred["classes"][i])+1]
            valid_detections.append(new_bb)
        
        return valid_detections

    def predict_positions(self, image:np.ndarray, confidence_limit=0.7) -> List[Named2DPosition]:
        pred = self._predict(image)
        valid_detections = list()

        for i,conf in enumerate(pred["confidence"]):
            if conf < confidence_limit:
                break

            new_pos = Named2DPosition()
            new_pos.x = int(image.shape[0]*pred["boxes"][i][1]) + (int(image.shape[0]*pred["boxes"][i][3]) - int(image.shape[0]*pred["boxes"][i][1]))/2
            new_pos.y = int(image.shape[0]*pred["boxes"][i][0]) + (int(image.shape[0]*pred["boxes"][i][2]) - int(image.shape[0]*pred["boxes"][i][0]))/2
            new_pos.name = self._label_map[int(pred["classes"][i])+1]
            valid_detections.append(new_pos)

        return valid_detections

    def draw_predictions(self, image:np.ndarray, confidence_limit=0.7) ->np.ndarray:
        boxes = self.preidict_bounds(image,confidence_limit)
        for box in boxes:
            pos1 = (int(box.vec_a.x), int(box.vec_a.y))
            pos2 = (int(box.vec_b.x), int(box.vec_b.y))
            image = cv2.rectangle(image.copy(), pos1, pos2, (0,255,0), 1)  
            text_pos_x = pos1[0]
            text_pos_y = pos1[1] - 10
            cv2.putText(image,box.label_name,(text_pos_x,text_pos_y),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1,cv2.LINE_AA)
        return image

class ObjectDetectorNode(Node):
    _bridge:CvBridge
    _object_detector:ObjectDetector
    _last_update: int
    _debug_view: bool

    def __init__(self) -> None:
        super().__init__('object_detector')

        self._bridge = CvBridge()
        self._load_params()
        self._init_detector()
        self._last_update = datetime.now().microsecond
        self._debug_view = self.get_parameter("debug_view").get_parameter_value().bool_value
        
        self.get_logger().info("Object Detector loaded")
        
        self.subscription = self.create_subscription(
            CompressedImage,
            self.get_parameter("image_topic").get_parameter_value().string_value,
            self.image_received_cb,
            HistoryPolicy.KEEP_LAST)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Subscription established")

    def _init_detector(self):
        detector_type = self.get_parameter("detector_type").get_parameter_value().string_value

        if detector_type == "tf":
            self._object_detector = ObjectDetector(
                model_path=self.get_parameter("model_path").get_parameter_value().string_value,
                label_map_path=self.get_parameter("label_map_path").get_parameter_value().string_value
            )
        elif detector_type == "aruco":
            detector_type = self.get_parameter("aruco_dict_type").get_parameter_value().string_value
            self._object_detector = ArucuDetector(detector_type)
        else:
            raise ValueError("unkown detector type")

    def _load_params(self) -> None:
        self.declare_parameter("detector_type","tf")
        self.declare_parameter("debug_view",False)
        self.declare_parameter("aruco_dict_type","DICT_4X4_100")
        self.declare_parameter("image_topic","cam_left/compressed")
        self.declare_parameter("model_path","/home/tobias/ros_docker/microwunderland-ros2/src/src_/object_detector/data/model.tf")
        self.declare_parameter("label_map_path","/home/tobias/ros_docker/microwunderland-ros2/src/src_/object_detector/data/label_map.pbtxt")

    def image_received_cb(self, msg:CompressedImage) -> None:
        if len(msg.data) == 0: return

        cv2Img = self._bridge.compressed_imgmsg_to_cv2(cmprs_img_msg=msg)
        img = np.asarray(cv2Img)
        img = self._object_detector.draw_predictions(img)
        img = cv2.resize(img,(1024,1024))
        cv2.imshow("processed_image",img) 
        cv2.waitKey(1)
        self.get_logger().info(f"FPS: {self._get_fps()}")

    def _get_fps(self) -> int:
        current_time = datetime.now().microsecond
        diff = (current_time - self._last_update)/1000
        fps = 1000 / diff
        self._last_update =current_time
        return fps

def main(args=None):
    rclpy.init(args=args)

    detector_node = ObjectDetectorNode()

    rclpy.spin(detector_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cv2.destroyAllWindows()
    detector_node.destroy_node()
    rclpy.shutdown()