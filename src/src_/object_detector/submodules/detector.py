import numpy as np
import tensorflow as tf
import cv2
from typing import Dict, List
from microwunderland_interfaces.msg import Named2DPosition
from microwunderland_interfaces.msg import BoundingBox

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
        self._label_map = read_label_map(label_map_path)
        self._interpreter = tf.lite.Interpreter(model_path)
        self._input_details = self._interpreter.get_input_details()
        self._output_details = self._interpreter.get_output_details()

        def read_label_map(label_map_path):

            item_id = None
            item_name = None
            items = {}

            with open(label_map_path, "r") as file:
                for line in file:
                    line.replace(" ", "")
                    if line == "item{":
                        pass
                    elif line == "}":
                        pass
                    elif "id" in line:
                        item_id = int(line.split(":", 1)[1].strip())
                    elif "display_name" in line:
                        item_name = line.split(" ")[-1].replace("\"", " ").strip()
                    if item_id is not None and item_name is not None:
                        items[item_name] = item_id
                        item_id = None
                        item_name = None

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
        self._interpreter.set_tensor(self._input_details[0]['index'], image)
        self._interpreter.invoke()
        predictions = dict()

        predictions["boxes"] = (self._interpreter.get_tensor(self._output_details[1]['index'])[0])
        predictions["classes"] = (self._interpreter.get_tensor(self._output_details[3]['index'])[0])
        predictions["confidence"] = (self._interpreter.get_tensor(self._output_details[0]['index'])[0])

        return predictions

    def preidict_bounds(self, image:np.ndarray, confidence_limit=0.7) -> List[BoundingBox]:
        pred = self._predict(image)
        valid_detections = list()

        for i,conf in enumerate(pred["confidence"]):
            if conf < confidence_limit:
                break

            new_bb = BoundingBox()
            new_bb.vec_a = Vector2D(int(image.shape[0]*pred["boxes"][i][1]), int(image.shape[1]*pred["boxes"][i][0]))
            new_bb.vec_b = Vector2D(int(image.shape[0]*pred["boxes"][i][3]), int(image.shape[1]*pred["boxes"][i][2]))
            new_bb.label = self._label_map[pred["classes"][i]]
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
            new_pos.name = self._label_map[pred["classes"][i]]

        return valid_detections
    
    