from microwunderland_interfaces.msg import ListNamed2DPositions
import rclpy
from rclpy.executors  import MultiThreadedExecutor
from rclpy.node  import Node
import yaml
from typing import Dict
import math
import numpy as np 
from submodules.traffic_planner_node import TrafficPlannerNode
from submodules.data_classes import Checkpoint,TrackedObject,MapNode,TravelPlan
import threading
from time import sleep


class TrafficPlanner(Node):

    _map:Dict[str,MapNode]
    _tracked_vehicles:Dict[str,TrackedObject]
    _ros_node: TrafficPlannerNode


    def __init__(self, plannerNode: TrafficPlannerNode):
        super().__init__('traffic_planner')
        self._ros_node = plannerNode

        self.load_params()

        self.load_map(self.get_parameter("map_location").get_parameter_value().string_value)
        self.get_logger().info("map loaded")

        self.load_travel_plan(self.get_parameter("travel_plan_location").get_parameter_value().string_value)
        self.get_logger().info("traffic_plan loaded")
        
        self.check_travel_plans()
        self.get_logger().info("traffic plans chcked")

        plannerNode.initialize_connections(
            switch_service_name= self.get_parameter("switch_service_name").get_parameter_value().string_value,
            tracker_topic= self.get_parameter("tracker_topic_name").get_parameter_value().string_value,
            tracked_vehicles=self._tracked_vehicles.keys()
        )


    def load_params(self) -> None:
        self.declare_parameter("map_location","./traffic_planer_data/map.yaml")
        self.declare_parameter("travel_plan_location","./traffic_planer_data/travel_plan.yaml")
        self.declare_parameter("switch_service_name","switch_controller")
        self.declare_parameter("tracker_topic_name","tracked_objects")

    
    def execute_travel_plan(self) -> None:
        self.enable_engines(True)
        self.get_logger().info("Engines startet")

        self.get_logger().info("start update loop")
        while True:
            try:
                self.update_vehicle_positions( self._ros_node.get_positions_from_buffer())
                for vehicle in self._tracked_vehicles.values():
                    self.check_vehicle_positions(vehicle)
            except KeyboardInterrupt:
                break
    

    def enable_engines(self, enable:bool):
        for vehicle in self._tracked_vehicles.values():
            self._ros_node.set_engine_state(vehicle.name,enable)
    

    def update_vehicle_positions(self, positions: ListNamed2DPositions) -> None:
        for pos in positions:
            self._tracked_vehicles[pos.name].position = [pos.x,pos.y]


    def check_vehicle_positions(self, vehicle: TrackedObject) -> None:
        next_switch = vehicle.travel_plan.checkpoints[vehicle.travel_plan.next_checkpoint]
        distToSwitch = math.dist(vehicle.position, next_switch.node.position)
        if distToSwitch <= next_switch.node.trigger_radius:
            self.get_logger().info(f"<{vehicle.name}> setting <{next_switch.node.name}> to state <{next_switch.state}>")
            
            self._ros_node.set_switch_state(next_switch.node.name, next_switch.state)    
            vehicle.travel_plan.next_checkpoint = (vehicle.travel_plan.next_checkpoint + 1) % len(vehicle.travel_plan.checkpoints)
            
            self.get_logger().info(f"<{vehicle.name}> next checkpoint is <{vehicle.travel_plan.checkpoints[vehicle.travel_plan.next_checkpoint].node.name}>")

            if vehicle.travel_plan.next_checkpoint == 0 and not vehicle.travel_plan.loop:
                self._ros_node.set_engine_state(vehicle.name,False)


    def load_map(self, path_to_map:str) -> None:
        self._map:Dict[str,MapNode] = dict()

        with open(path_to_map, 'r') as file:
            map_dict = yaml.safe_load(file)
        
        for node in map_dict["nodes"]:
            pos = np.array([
                node["positions"]["x"],
                node["positions"]["y"]
            ])

            newNode = MapNode(
                name = node["node_name"],
                position= pos,
                trigger_radius= node["trigger_radius"],
                connections= node["connections"]
            )

            self._map[newNode.name] = newNode
    

    def load_travel_plan(self, path_to_plan:str) -> None:
        self._tracked_vehicles:Dict[str,TrackedObject] = dict()

        with open(path_to_plan, 'r') as file:
            config_dict = yaml.safe_load(file)

        for item in config_dict["travel_plans"]:
            try:
                checkpoints =  list(map(
                    lambda checkpoint: 
                        Checkpoint(self._map[checkpoint["switch_name"]],checkpoint["switch_state"]),
                        item["checkpoints"]
                ))
            except KeyError as ex:
                raise Exception(f"the checkpoint {ex.args[0]} does not exist in the map")

            travel_plan = TravelPlan(
                start_time = item["start_time"],
                loop = item["loop_plan"],
                next_checkpoint=0,
                checkpoints=checkpoints
            )

            vehicle = TrackedObject(
                name=item["vehicle_name"],
                position=np.array([0.0, 0.0]),
                travel_plan=travel_plan
            )
            self._tracked_vehicles[vehicle.name] = vehicle
        
        
    def check_travel_plans(self) -> None:
        for vehicle in self._tracked_vehicles.values():
            for index,checkpoint in enumerate(vehicle.travel_plan.checkpoints):               
                if index+1 < len(vehicle.travel_plan.checkpoints):
                    self.check_if_connected(checkpoint,vehicle.travel_plan.checkpoints[index+1])
                elif vehicle.travel_plan.loop:
                    self.check_if_connected(checkpoint,vehicle.travel_plan.checkpoints[0])


    def check_if_connected(self, a:Checkpoint, b:Checkpoint) -> None:
        if b.node.name not in a.node.connections:
            raise Exception(f"[{a.node.name}] is not connected with [{b.node.name}]!")


from threading import Condition

cv = Condition()
available = False
def a():
    while True:
        global cv
        global available
        with cv:
            while available<15:
                cv.wait()
            available = 0
            print(f"consume {available}")

def b():
    while True:
        sleep(1)
        global cv
        global available
        # Produce one item
        with cv:
            available += 1
            print(f"produce {available}")
            cv.notify()


def main(args=None):
    rclpy.init(args=args)

    network_node = TrafficPlannerNode()
    planner_node = TrafficPlanner(network_node)

    t1 = threading.Thread(target= rclpy.spin, args=[network_node], daemon=True)
    t1.start()
    planner_node.execute_travel_plan()

    t1.join()
    rclpy.shutdown()

if __name__ == '__main__':
    main()