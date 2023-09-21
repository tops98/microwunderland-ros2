import numpy as np
import tensorflow as tf
import cv2
import json


# load tf-lite model
tflite_model_path = "tflite.tf"
interpreter = tf.lite.Interpreter(model_path=tflite_model_path)
interpreter.allocate_tensors()

# load  input and output details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

mean = 127.5
std = 127.5


def read_anotation_dict(path:str) -> dict:
  with open(path,'+r') as file:
    return json.load(file)

working_dir = "/home/tobias/ros_docker/microwunderland-ros2/src/KalmanFilterTest/evaluate_object_detector/microwunderland.v1i.coco/valid/"
annotations_dict = read_anotation_dict(working_dir+"_annotations.coco.json")

for picture in annotations_dict["images"]:
  # load image
  path = working_dir+picture["file_name"]
  image = cv2.imread(path)
  input_data = np.expand_dims(image,axis=0)
  normalized_image_array = (np.float32(input_data) - mean)/std
  print(input_details[0]['dtype'])
  print(input_details[0]['shape'])


  # predict
  interpreter.set_tensor(input_details[0]['index'], normalized_image_array)
  interpreter.invoke()

  # safe predictions
  boxes = (interpreter.get_tensor(output_details[1]['index'])[0])
  classes = (interpreter.get_tensor(output_details[3]['index'])[0])
  confidence = (interpreter.get_tensor(output_details[0]['index'])[0])

  results = [boxes,classes,confidence]
  print(results)

  # draw bounding boxes
  image = cv2.imread(path)
  threshold = 0.7
  color = (0,255,0)
  line_thickness = 1
  image_shape = image.shape

  for i in range(0,len(results[1])):
    if(results[2][i] < threshold):
      break

    pos1 = ( int(image_shape[0]*results[0][i][1]), int(image_shape[1]*results[0][i][0]))
    pos2 = ( int(image_shape[0]*results[0][i][3]), int(image_shape[1]*results[0][i][2]))
    cv2.rectangle(image, pos1, pos2, color, line_thickness)

  for annotation in annotations_dict["annotations"]:
    if annotation["image_id"] == picture["id"]:
      bbox = annotation["bbox"]
      pos1 = int(bbox[0]),int(bbox[1])
      pos2 = int(bbox[0])+int( bbox[2]),int(bbox[1])+int( bbox[3])
      cv2.rectangle(image, pos1, pos2, (255,0,0), 2)
      print(f"image id= {picture['id']} pos1 = {pos1}, pos2 = {pos2}")

  cv2.imshow("processed_image",image) 
  cv2.waitKey(0)
  cv2.destroyAllWindows()