import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from microwunderland_interfaces.msg import TrackedObjects, TrackedObject


class VisualizerNode(Node):

    def __init__(self):
        super().__init__('visualizer_node')

        self._load_params()
        self._bridge = CvBridge()
        self._last_img:np.ndarray = None

        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        tracker_topic = self.get_parameter("tracker_topic").get_parameter_value().string_value

        self._publish_visualization = self.get_parameter("publish_visualization").get_parameter_value().bool_value
        self._draw_visualization = self.get_parameter("draw_visualization").get_parameter_value().bool_value
        self._max_delta = self.get_parameter("delta_max").get_parameter_value().double_value

        self.get_logger().info(f"draw_visualization = {self._draw_visualization}")
        self.get_logger().info(f"publish_visualization = {self._publish_visualization}")
        self.get_logger().info(f"delta_max = {self._max_delta}")

        self.subscription = self.create_subscription(
            CompressedImage,
            image_topic,
            self.image_received_cb,
            HistoryPolicy.KEEP_LAST,
            callback_group= MutuallyExclusiveCallbackGroup()
        )

        self.subscription = self.create_subscription(
            TrackedObjects,
            tracker_topic,
            self.tracker_received_cb,
            HistoryPolicy.KEEP_LAST,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self._publisher = self.create_publisher(
            CompressedImage,
            output_topic,
            HistoryPolicy.KEEP_LAST
        )

        self.get_logger().info("Tracker Visualizer ready")
    
    def image_received_cb(self, msg) -> None:
        if len(msg.data) == 0: return

        cv2Img = self._bridge.compressed_imgmsg_to_cv2(cmprs_img_msg=msg)
        self._last_img = np.asarray(cv2Img)

    def tracker_received_cb(self, msg) -> None:
        if len(msg.objects) == 0 or self._last_img is None: return

        image = self._last_img.copy()
        for object in msg.objects:
            image = self.draw_object_annotation(image, object)

        if self._publish_visualization:
            msg = self._bridge.cv2_to_compressed_imgmsg(image)
            self._publisher.publish(msg)
        if self._draw_visualization:
            cv2.imshow("processed_image",image) 
            cv2.resizeWindow("processed_image",400,400)
            cv2.waitKey(1)
            
            
    def draw_object_annotation(self, image:np.ndarray, object:TrackedObject) -> np.ndarray:
        
        x = int(object.position.x)
        y = int(object.position.y)

        color = (0,255,0)
        if object.undetected:
            color = (0,0,255)

        image = cv2.circle(image, (x, y), int(self._max_delta), (0,255,255), 1)  
        image = cv2.circle(image, (x, y), 2, color, 2)  
        image = cv2.putText(image, "ID_"+str(object.id),(x+10, y+10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),1,cv2.LINE_AA)

        return image

    def _load_params(self):
        self.declare_parameter("image_topic","cam_left/compressed")
        self.declare_parameter("tracker_topic","tracked_objects")
        self.declare_parameter("delta_max",20.)
        self.declare_parameter("draw_visualization",False)
        self.declare_parameter("publish_visualization",True)
        self.declare_parameter("output_topic","tracker_visualized")


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    visualizer = VisualizerNode()
    executor.add_node(visualizer)

    executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    visualizer.destroy_node()
    executor.shutdown()


if __name__ == '__main__':
    main()