import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from pathlib import Path
import cv2
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
import sys


class ImageExporterNode(Node):

    _count: int
    _num_exported: int
    _export_path: str
    _image_stream_topic: str
    _buffer: CompressedImage
    _last_buff_val: CompressedImage
    _nth_frame: int
    _time_intervall:float


    def __init__(self):
        super().__init__('image_exporter')
        self.get_logger().info("starting node")

        self._count = 0
        self._num_exported = 0
        self._bridge = CvBridge()
        self._buffer = None
        self._last_buff_val = None

        self.declare_parameter("topic_name","cam_left/compressed")
        self.declare_parameter("export_path","image_exports")
        self.declare_parameter("nth_sample",1)
        self.declare_parameter("time_intervall",0.0)
        self.declare_parameter("max_time",0.0)
        self.declare_parameter("max_exports",0)


        self._max_time = self.get_parameter("max_time").get_parameter_value().double_value
        self._max_exports = self.get_parameter("max_exports").get_parameter_value().integer_value

        self._time_intervall = self.get_parameter("time_intervall").get_parameter_value().double_value
        self._nth_frame = self.get_parameter("nth_sample").get_parameter_value().integer_value

        self._export_path = self.get_parameter("export_path").get_parameter_value().string_value
        self._image_stream_topic = self.get_parameter("topic_name").get_parameter_value().string_value

        if self._max_time > 0:
            self.get_logger().info(f"setting max execution time to {self._max_time} seconds")
            self.create_timer(self._max_time,self.shut_down)

        if self._time_intervall > 0:
            self.get_logger().info(f"setting intervall to {self._time_intervall} seconds")
            self.create_timer(self._time_intervall,self.timer_cb)


        self.create_subscription(CompressedImage,self._image_stream_topic,self.img_received_cb,10)
        self.get_logger().info(f"subscribed to topic: {self._image_stream_topic}")


    def timer_cb(self):
        self.export_image(self._export_path, self._buffer)


    def img_received_cb(self, msg:CompressedImage) -> None:
        if self._time_intervall > 0:
            self._buffer = msg
        else:
            self.export_image(self._export_path,msg)
        

    def export_image(self, path:str, msg:CompressedImage) -> None:

        if self.check_if_export_allowed(msg):

            self.get_logger().info("exporting...")
            Path(self._export_path).mkdir(parents=True,exist_ok=True)
            export_path = f"{self._export_path}/{datetime.now()}.jpg"    
            cv2Img = self._bridge.compressed_imgmsg_to_cv2(cmprs_img_msg=msg)
            cv2.imwrite(export_path, cv2Img)
            self._num_exported += 1
            self.get_logger().info(f"wrote image to: {export_path}")

        if self._num_exported == self._max_exports:
            self.shut_down()


    def shut_down(self) -> None:
        self.destroy_node()
        sys.exit()
        

    def check_if_export_allowed(self, img_msg:CompressedImage) -> bool:
        self._count = (self._count +1) % self._nth_frame

        if self._count == 0 and img_msg != None and img_msg != self._last_buff_val:
            self._last_buff_val = img_msg
            return True
        return False




def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ImageExporterNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()