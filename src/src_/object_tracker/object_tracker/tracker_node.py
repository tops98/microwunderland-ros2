import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import HistoryPolicy
from object_tracker.submodules.centroid_tracker import CentroidTracker, Centroid
from object_tracker.submodules.kalman_filter import CVD_KF_Config
from typing import Tuple,List
import numpy as np
from microwunderland_interfaces.msg import BoundingBoxList,TrackedObject,TrackedObjects, Vector2D


class TrackerNode(Node):

    def __init__(self):
        super().__init__('tracker_node')

        self._load_params()

        # get params for filter
        intial_uncertainty = self.get_parameter('estimatation_uncertainty').get_parameter_value().double_array_value
        measurment_uncertainty = self.get_parameter('measurment_uncertainty').get_parameter_value().double_array_value
        processnoise = self.get_parameter('processnoise').get_parameter_value().double_array_value
        num_axis = self.get_parameter('num_axis').get_parameter_value().integer_value
        # get params for tracker
        delta_max = self.get_parameter("delta_max").get_parameter_value()._double_value
        num_probation_updates = self.get_parameter("num_probation_updates").get_parameter_value().integer_value
        max_update_pause_ms=self.get_parameter("max_update_pause").get_parameter_value()._double_value
        # get params for node
        detection_topic = self.get_parameter("detections_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._sample_rate = 1.0 / self.get_parameter("sample_rate").get_parameter_value()._double_value

        filter_config = CVD_KF_Config(
            num_measurements= num_axis,
            initial_uncertenty= np.array(intial_uncertainty),
            measurment_uncertenty= np.array(measurment_uncertainty),
            process_noise= np.array(processnoise)
        )

        self._tracker = CentroidTracker(
            delta_max,
            max_update_pause_ms,
            filter_config,
            num_probation_updates= num_probation_updates
        )

        self._publisher = self.create_publisher(
            TrackedObjects,
            output_topic,
            HistoryPolicy.KEEP_LAST
        )

        self.create_subscription(
            BoundingBoxList,
            detection_topic,
            self.listener_callback,
            HistoryPolicy.KEEP_LAST,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.create_timer(self._sample_rate, self.publish_predictions,MutuallyExclusiveCallbackGroup())
        self.get_logger().info("Object Tracker ready")
        self.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'processnoise' and param.type_ == Parameter.Type.DOUBLE_ARRAY:

                q_matrix = self._create_covarianz_matrix(np.array(param.value))
                for centroid in self._tracker._centroids:
                    centroid._filter._Q = q_matrix
                    
        return SetParametersResult(successful=True)
    
    def _create_covarianz_matrix(self, array:np.ndarray) -> np.ndarray:

        offset = array.size // 3
        cov_matrix = np.zeros((array.size,array.size))
        for y in range(array.size):
            velocity_colum = (y+offset) % array.size
            accelaration_colum = (y+2*offset) % array.size
            
            cov_matrix[y,y] = array[y]**2
            cov_matrix[y,velocity_colum] = array[y]*array[velocity_colum] 
            cov_matrix[y,accelaration_colum] = array[y]*array[accelaration_colum]
        
        return cov_matrix

    def _load_params(self) -> None:
        self.declare_parameter("detections_topic","predictions_cam_left")
        self.declare_parameter("output_topic","tracked_objects")
        self.declare_parameter("sample_rate",10.0)
        
        self.declare_parameter("delta_max", 30.0)
        self.declare_parameter("num_probation_updates", 5)
        self.declare_parameter("max_update_pause", 6000.)

        self.declare_parameter("num_axis", 2)
        self.declare_parameter('estimatation_uncertainty',[100.,100.,100.,100.,100.,100.])
        self.declare_parameter('measurment_uncertainty',[0.25,0.25])
        self.declare_parameter('processnoise',[.9, .9, 0.6, 0.6, 0.1, 0.1])   

    def publish_predictions(self):
        if len(self._tracker.get_state) > 0:
            msg = self.create_message(self._tracker.get_state)
            self._publisher.publish(msg)
        
    def create_message(self, centroids:List[Centroid]) -> TrackedObjects:
        tracked_objects = TrackedObjects()
        for centroid in centroids:
            if centroid.number_of_updates < self._tracker._num_probation_updates:
                continue
            
            object = TrackedObject()
            position = Vector2D()
            velocity = Vector2D()
            acceleration = Vector2D()
            centroid.predict(delta_time=self._sample_rate)

            position.x = centroid.position[0]
            position.y = centroid.position[1]

            velocity.x = centroid.velocity[0]
            velocity.y = centroid.velocity[1]

            acceleration.x = centroid.acceleration[0]
            acceleration.y = centroid.acceleration[1]
            

            object.position= position
            object.velocity= velocity
            object.acceleration= acceleration

            object.id = centroid.id
            object.undetected = centroid.unmatched

            tracked_objects.objects.append(object)

        return tracked_objects

    def listener_callback(self, msg):
        bounding_boxes = msg.boxes
        positions = np.zeros((len(bounding_boxes),2))

        for i,bb in enumerate(bounding_boxes):
            positions[i,:] = self._get_center(
                x1=bb.pos1.x,
                x2=bb.pos2.x,
                y1=bb.pos1.y,
                y2=bb.pos2.y
            )
        self._tracker.update(positions)

    def _get_center(self, x1:float, y1:float, x2:float, y2:float) -> Tuple[float,float]:
        x = (x1 + x2) / 2
        y = (y1 + y2) / 2
        return (x,y)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = TrackerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(minimal_subscriber)
    executor.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    executor.shutdown()


if __name__ == '__main__':
    main()