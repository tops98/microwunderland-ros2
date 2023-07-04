import numpy as np
import tensorflow as tf
import cv2

# load tf-lite model
tflite_model_path = "tflite.tf"
interpreter = tf.lite.Interpreter(model_path=tflite_model_path)
interpreter.allocate_tensors()

# load  input and output details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# load image
mean = 127.5
std = 127.5
path = "valid/96_jpg.rf.15494bf46bd50f4690022f66dfef10c0.jpg"
image = cv2.imread(path)
input_data = np.expand_dims(image,axis=0)
normalized_image_array = (np.float32(input_data) - mean)/std
print(input_details[0]['dtype'])

# predict
interpreter.set_tensor(input_details[0]['index'], normalized_image_array)
interpreter.invoke()

# safe predictions
boxes = (interpreter.get_tensor(output_details[1]['index'])[0])
classes = (interpreter.get_tensor(output_details[3]['index'])[0])
confidence = (interpreter.get_tensor(output_details[0]['index'])[0])

results = [boxes,classes,confidence]

# draw bounding boxes
image = cv2.imread(path)
threshold = 0.7
color = (0,255,0)
line_thickness = 1
image_shape = image.shape
image = cv2.rotate(image,rotateCode = cv2.ROTATE_90_CLOCKWISE)
image = cv2.flip(image,1)

for i in range(0,len(results[1])):
  if(results[2][i] < threshold):
    break

  pos1 = ( int(image_shape[0]*results[0][i][0]), int(image_shape[1]*results[0][i][1]))
  pos2 = ( int(image_shape[0]*results[0][i][2]), int(image_shape[1]*results[0][i][3]))
  cv2.rectangle(image, pos1, pos2, color, line_thickness)

print(i)
cv2.imshow("processed_image",image) 
cv2.waitKey(0)
cv2.destroyAllWindows()
