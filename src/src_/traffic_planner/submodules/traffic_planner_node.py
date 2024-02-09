from microwunderland_interfaces.msg import TrackedObjects
from microwunderland_interfaces.srv import SetState
from std_srvs.srv import SetBool
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.subscription import Subscription
from rclpy.node import Node,Client
from typing import List,Dict
from threading import Condition
import threading
from time import sleep


class TrafficPlannerNode(Node):

    _buffer_cond: Condition
    _tracked_obj_buffer: TrackedObjects
    _switch_service: Client
    _vehicle_services: Dict[str,Client]
    _tracker_sub: Subscription
    _switch_cb_group: ReentrantCallbackGroup
    _vehicles_cb_group: ReentrantCallbackGroup
    _tracker_cb_group: ReentrantCallbackGroup


    def __init__(self):
        super().__init__('traffic_planner_networker')
        self._vehicle_services = dict()
        self._switch_service = None
        self._tracker_sub = None
        self._tracked_obj_buffer = None
        self._buffer_cond = Condition()

        self._switch_cb_group = ReentrantCallbackGroup()
        self._vehicles_cb_group = ReentrantCallbackGroup()
        self._tracker_cb_group =ReentrantCallbackGroup()
        

    def initialize_connections(self, switch_service_name:str, tracker_topic:str, tracked_vehicles: List[str]) -> None:
        self._subscribe_to_switch_services(f'{switch_service_name}/set_state')
        self.get_logger().info("subscribed to switch service")
        
        self._subscribe_to_vehicle_service(tracked_vehicles)
        self.get_logger().info("subscribed to car service")
        
        self._subscribe_to_tracker(tracker_topic)
        self.get_logger().info("subscribed to tracker service")



    def add_positions_to_buffer(self, positions: TrackedObjects) -> None:
        with self._buffer_cond:
            self._tracked_obj_buffer = positions
            self._buffer_cond.notify()


    def get_positions_from_buffer(self) -> TrackedObjects:
        with self._buffer_cond:
            while self._tracked_obj_buffer == None:
                self._buffer_cond.wait()

            pos = self._tracked_obj_buffer
            self._tracked_obj_buffer = None
        return pos
    

    def set_switch_state(self, switch_name:str, state_name:str) -> None:
        req = SetState.Request()
        req.switch_name = switch_name
        req.state_name = state_name

        result = self._switch_service.call(req)
        if result.status != 0:
            self.get_logger().error(f"switch_service is not responding!")


    def set_engine_state(self, vehicle_name:str, state: bool) -> None:
        req = SetBool.Request()
        req.data = state

        result = self._vehicle_services[vehicle_name].call(req)
        if not result.success:
            self.get_logger().error(f"vehicle [{vehicle_name}] is not responding!")
    

    def _subscribe_to_tracker(self, topic_name:str) -> None:
        if self._tracker_sub:
            self.destroy_subscription(self._tracker_sub)
        
        self._tracker_sub = self.create_subscription(
            TrackedObjects,
            topic_name,
            self._tracker_listener_callback,
            10,
            callback_group=self._tracker_cb_group)
        
        
    def _subscribe_to_vehicle_service(self, vehicel_names:List[str]) -> None:
        if len(self._vehicle_services)>0:
            for srv in self._vehicle_services.values():
                self.destroy_client(srv)

        self._vehicle_services: Dict[str,Client] = dict()

        for vehicle in vehicel_names:    
            service = self.create_client(SetBool, f'{vehicle}/set_engine_state',callback_group= self._vehicles_cb_group)
            while not service.wait_for_service(timeout_sec=1.0):
                self.get_logger().warning(f'service for vehicle [{vehicle}] not online')    
            self._vehicle_services[vehicle] = service
    
    def _subscribe_to_switch_services(self, service_name:str) -> None:
        if self._switch_service:
            self.destroy_client(self._switch_service)

        self._switch_service: Client = self.create_client(SetState, service_name,callback_group= self._switch_cb_group)
        while not self._switch_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f'switch_service is not online!')


    def _tracker_listener_callback(self, msg):
        self.add_positions_to_buffer(msg.named_positions)