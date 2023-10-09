import cv2
import numpy as np
import re

import tensorflow as tf
from typing import Dict,Tuple

class ObjectDetector:
    _interpreter:tf.lite.Interpreter
    _input_details:list
    _output_details:list
    _label_map: Dict
    _mean: float
    _std: float
    _confidence_limit:float

    def __init__(self, model_path:str, label_map_path:str, confidence_limit:float=0.8, std:float=127.5, mean:float=127.5) -> None:
        self._mean = mean
        self._std = std
        self._confidence_limit = confidence_limit
        self._label_map = self._read_label_map(label_map_path)
        self._interpreter = tf.lite.Interpreter(model_path)
        self._interpreter.allocate_tensors()
        self._input_details = self._interpreter.get_input_details()
        self._output_details = self._interpreter.get_output_details()

    def _read_label_map(self, label_map_path) -> Dict[int,str]:
        items = {}

        with open(label_map_path, "r") as file:
            pattern = r"(item\s*{[^}]+})"
            matches = re.findall(pattern,file.read())
           
            if matches is  None or len(matches)<1:
                raise Exception("lable map does not contain any items: please check if its formated correctly")
            
            for match in matches:
                
                id_match = re.search(r"(id\s*:\s*)(\d+)",match)
                display_name_match = re.search(r'(display_name\s*:\s*")(\w+)',match)
                name_match = re.search(r'(name\s*:\s*")(\w+)',match)
                
                if id_match is None:
                    raise Exception(f"no id found in item:\n{match}")
                id = int(id_match[2])
                if id in items.keys():
                    raise Exception("duplicate id detected")
                
                if name_match is not None:
                    items[id] = name_match[2]
                elif display_name_match is not None:
                    items[id] = display_name_match[2]
                else:
                    raise Exception(f"no name or display name found in item:\n{match}")
                  
        return items

    def _preprocess_image(self, image:np.ndarray, image_dim:Tuple[int,int]=None) -> np.ndarray:
        # resize image to input shape of model
        if image_dim is None:
            image_dim = self._input_details[0]['shape'][1:3]
        image = cv2.resize(image, image_dim)
        # add axis, since model usally expects batches
        image = np.expand_dims(image,axis=0)
        # standardize data if float input is used
        if self._input_details[0]['dtype'] == np.float32:
            image = (np.float32(image) - self._mean)/self._std
        return image
    
    def get_label_for_class(self, class_id:int):
        return self._label_map.get(class_id,None)
    
    def predict(self, image:np.ndarray) -> Dict[str,any]:
        processedImg = self._preprocess_image(image)
        self._interpreter.set_tensor(self._input_details[0]['index'], processedImg)
        self._interpreter.invoke()

        predictions_filtered = list()
        confidence_scores = (self._interpreter.get_tensor(self._output_details[0]['index'])[0])
        boxes = (self._interpreter.get_tensor(self._output_details[1]['index'])[0])
        classes = (self._interpreter.get_tensor(self._output_details[3]['index'])[0])

        for i,confidence in enumerate(confidence_scores):
            if confidence > self._confidence_limit:
                box = self._convert_box_coords_to_pixel_coords(boxes[i])
                predicted_class = classes[i]
                predictions_filtered.append((predicted_class,box))

        return predictions_filtered

    
    def _convert_box_coords_to_pixel_coords(self, box_coords):
        image_shape = self._input_details[0]['shape'][1:3]

        x_1 =(image_shape[0]*box_coords[1])
        y_1 =(image_shape[1]*box_coords[0])
                                
        x_2 =(image_shape[0]*box_coords[3])
        y_2 =(image_shape[1]*box_coords[2])

        return (x_1, y_1, x_2, y_2)


    # def draw_predictions(self, image:np.ndarray, prediction) ->np.ndarray:
    #     boxes, classes  = prediction
    #     for index,box in enumerate(boxes):
    #         pos1 = (int(box[0]), int(box[2]))
    #         pos2 = (int(box[1]), int(box[3]))
    #         image = cv2.rectangle(image.copy(), pos1, pos2, (0,255,0), 1)  
    #         text_pos_x = pos1[0]
    #         text_pos_y = pos1[1] - 10
    #         cv2.putText(image,self._label_map[int(classes[index])+1],(text_pos_x,text_pos_y),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1,cv2.LINE_AA)
    #     return image
