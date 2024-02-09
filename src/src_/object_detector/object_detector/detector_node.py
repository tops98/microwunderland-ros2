from typing import List, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from microwunderland_interfaces.msg import BoundingBoxList,BoundingBox
from object_detector.submodules.object_detector import ObjectDetector
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from sensor_msgs.msg import CompressedImage
from pprint import pprint


class ObjectDetectorNode(Node):
    _bridge:CvBridge
    _object_detector:ObjectDetector

    def __init__(self) -> None:
        super().__init__('object_detector')

        self._load_params()
        self._bridge = CvBridge()
        
        self._object_detector = ObjectDetector(
            model_path= self.get_parameter("model_path").get_parameter_value().string_value,
            label_map_path= self.get_parameter("label_map_path").get_parameter_value().string_value,
            confidence_limit= self.get_parameter("confidence_limit").get_parameter_value()._double_value,
            mean= self.get_parameter("mean").get_parameter_value()._double_value,
            std= self.get_parameter("std").get_parameter_value()._double_value,
        )

        self._publisher = self.create_publisher(
            BoundingBoxList,
            self.get_parameter("output_topic").get_parameter_value().string_value,
            HistoryPolicy.KEEP_LAST
        )        

        self.subscription = self.create_subscription(
            CompressedImage,
            self.get_parameter("image_topic").get_parameter_value().string_value,
            self.image_received_cb,
            HistoryPolicy.KEEP_LAST
        )
        
        self.get_logger().info("Object Detector ready")


    def _load_params(self) -> None:
        # automaticly find path to working directory relative to script location
        ws_path = '/'.join(str(__file__).split('/')[0:-2])

        self.declare_parameter("output_topic","predictions_cam_left")
        self.declare_parameter("image_topic","cam_left/compressed")

        self.declare_parameter("model_path",ws_path+"/data/SSD_MobileNet_V2_FPNLite_320x320.tf")
        self.declare_parameter("label_map_path",ws_path+"/data/label_map.pbtxt")

        self.declare_parameter("confidence_limit",0.75)
        self.declare_parameter("mean",127.5)
        self.declare_parameter("std",127.5)

    def image_received_cb(self, msg:CompressedImage) -> None:
        if len(msg.data) == 0: return

        cv2Img = self._bridge.compressed_imgmsg_to_cv2(cmprs_img_msg=msg)
        img = np.asarray(cv2Img)
        predictions = self._object_detector.predict(img)
        msg = self._convert_to_ros_msg(predictions)

        self._publisher.publish(msg)
        
    def _convert_to_ros_msg(self, predictions:List[Tuple[float,List[float]]]) -> BoundingBoxList:

        bb_array = BoundingBoxList()
        for prediction in predictions:

            print(prediction)
            print(prediction[1][0])
            bb = BoundingBox()
            bb.pos1.x = float(prediction[1][0]) # y1
            bb.pos1.y = float(prediction[1][1]) # x1

            bb.pos2.x = float(prediction[1][2]) # y2
            bb.pos2.y = float(prediction[1][3]) # x2
            bb.label = "ID_"+str(prediction[0])
            bb_array.boxes.append(bb)

        return bb_array
    

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

if __name__ == '__main__':
    main()