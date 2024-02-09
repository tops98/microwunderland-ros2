import unittest
from object_detector import ObjectDetector
import cv2 
import numpy as np

class TestObjectDetector(unittest.TestCase):

    def __init__(self, methodName: str = "runTest") -> None:
        super().__init__(methodName)
        self._data_path = "/home/ros2_workspace/src_/object_detector/data/"
        model_name = "SSD_MobileNet_V2_FPNLite_320x320.tf"
        label_map_name = "label_map.pbtxt"

        self._model_path = self._data_path+model_name
        self._label_map_path = self._data_path+ label_map_name

    # Test load_label_map
    def test_load_label_map(self):
        normal = self._data_path+"test_label_map.pbtxt"
        detector = ObjectDetector( self._model_path, normal)
        
        labels =  detector._read_label_map(normal)
        self.assertEqual("Car",labels[1])
        self.assertEqual("Plane",labels[2])
        self.assertEqual("Ship",labels[3])
        self.assertEqual("Horse",labels[4])
    
    def test_label_map_no_id(self):
        no_id = self._data_path+"test_label_map_no_id.pbtxt"        
        self.assertRaises(Exception, ObjectDetector._read_label_map,no_id)

    def test_label_map_no_name(self):
        no_name = self._data_path+"test_label_map_no_name.pbtxt"
        self.assertRaises(Exception, ObjectDetector._read_label_map, no_name)

    def test_label_map_no_item(self):
        no_item = self._data_path+"test_label_map_no_item.pbtxt"
        self.assertRaises(Exception, ObjectDetector._read_label_map, no_item)
        
    def test_label_map_no_dublicate_id(self):
        dublicated_id = self._data_path+"test_label_map_dublicate_id.pbtxt"        
        self.assertRaises(Exception, ObjectDetector._read_label_map, dublicated_id)

    # Test image preprocessing  
    def test_preprocess_image_resize(self):
        image = cv2.imread(self._data_path+"test_image.jpg")
        detector = ObjectDetector( self._model_path, self._label_map_path)
        shapes = [(32,32),(1024,1024),(320,320),(320,620),(620,320)]
        
        for current_shape in shapes:
            img_shape = detector._preprocess_image(image,current_shape).shape[1:3]
            self.assertEqual((current_shape[1],current_shape[0]),img_shape)

    def test_preprocess_image_add_axis(self):
        image = cv2.imread(self._data_path+"test_image.jpg")
        detector = ObjectDetector( self._model_path, self._label_map_path)

        img_shape = detector._preprocess_image(image).shape
        self.assertEqual(4,len(img_shape))
        self.assertEqual(1,img_shape[0])

        
    # Test prediction
    def test_predictions(self):
        expected = np.array([[0.0,130.51335334777832, 49.66021537780762, 179.4292449951172, 85.9719467163086],
        [0.0, 273.2653617858887, 195.21209716796875, 304.1395378112793, 228.79135131835938],
        [0.0, 272.09957122802734, 215.5615234375, 303.6751937866211, 247.70103454589844],
        [0.0, 279.60567474365234, 32.25118160247803, 313.10096740722656, 57.58814334869385],
        [0.0, 271.09508514404297, 174.81225967407227, 296.70082092285156, 208.0788230895996]])
        image = cv2.imread(self._data_path+"test_image.jpg")
        detector = ObjectDetector( self._model_path, self._label_map_path)
        predictions = detector.predict(image)
        
        for index,pred in enumerate(predictions):
            # check class
            self.assertAlmostEqual(expected[index,0],pred[0])
            # check pos
            self.assertAlmostEqual(expected[index,1],pred[1][0])
            self.assertAlmostEqual(expected[index,2],pred[1][1])
            self.assertAlmostEqual(expected[index,3],pred[1][2])
            self.assertAlmostEqual(expected[index,4],pred[1][3])


if __name__ == "__main__":
    unittest.main()
