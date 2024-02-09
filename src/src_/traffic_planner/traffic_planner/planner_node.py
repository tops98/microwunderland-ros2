import rclpy
from rclpy.node import Node
from rclpy.node import Client
from rclpy.qos import HistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup,ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from schema import SchemaError
import yaml
import numpy as np
from submodules.yaml_schemas import map_schema,travel_plan_schema
from submodules.data_classes import MapNode, TravelPlan, ActiveTravelPlan, Checkpoint
from submodules.data_classes import TinycarService, TaillightStates, BlinkerStates, HeadlightStates 
from typing import Dict
from microwunderland_interfaces.srv import SetString, StringArray, AssigneTravelPlan, SetState, SetUint32KeyValue, GetFloat32
from microwunderland_interfaces.msg import TrackedObject, TrackedObjects


# TODO:
# + Was passiert wenn ein Fahrzeug mit aktiven Fahrplan einen neuen Plan zugewiesen bekommt?
# + Automatische Assoziation von Tinycar und getrackeden Fahrzeug
# + Weichen können als Aktion unabhängig vom Chcekpoint gestellt werden
# + Entkoppelung von checkpoints und Mapnodes (freie Angabe von Positionen mit triggerradius)
class Planner(Node):

    _active_plans: Dict[str,ActiveTravelPlan]
    _switch_client: Client
    _tinycar_services: TinycarService
    _map:Dict[str,MapNode]
    _travel_plans:Dict[str,TravelPlan]


    def __init__(self) -> None:
        super().__init__('traffic_planner')
        self._active_plans = dict()
        self._load_params()
        service_server_cb_group = MutuallyExclusiveCallbackGroup()
        tracker_update_cb_group = MutuallyExclusiveCallbackGroup()
        service_call_cb_group = MutuallyExclusiveCallbackGroup()

        map_path = self.get_parameter("map_path").get_parameter_value().string_value
        travel_plan_path = self.get_parameter("travel_plans_path").get_parameter_value().string_value
        
        with open(map_path,'r') as file:    
            self._map = self._parse_map(file.read())
        
        with open(travel_plan_path,'r') as file:
            self._travel_plans = dict()    
            self._parse_travel_plan(file.read(), raise_error_if_invalid=True)
       
        # create service clients
        #   switch service
        self._switch_client = self.create_client(SetState, "set_state", callback_group=service_call_cb_group)
        #   tinycar_adapter
        self._tinycar_services = TinycarService(
            self.create_client(GetFloat32,"get_battery_status", callback_group=service_call_cb_group),
            self.create_client(SetUint32KeyValue, "set_speed", callback_group=service_call_cb_group),
            self.create_client(SetUint32KeyValue, "set_head_light", callback_group=service_call_cb_group),
            self.create_client(SetUint32KeyValue, "set_tail_light", callback_group=service_call_cb_group),
            self.create_client(SetUint32KeyValue, "set_blinker", callback_group=service_call_cb_group)
        )

        # create tracker subscriptions
        tracker_topic = self.get_parameter("tracker_topic").get_parameter_value().string_value
        self.create_subscription(TrackedObjects, tracker_topic, self._tracker_cb, HistoryPolicy.KEEP_LAST, callback_group=tracker_update_cb_group)

        # create services servers
        self.create_service(AssigneTravelPlan, "AssigneTravelPlan", self._assigne_travel_plan_cb, callback_group=service_server_cb_group)
        self.create_service(SetString, "AddTravelPlan", self._add_travel_plan_cb, callback_group=service_server_cb_group)
        self.create_service(StringArray, "Start", self._start_cb, callback_group=service_server_cb_group)
        self.create_service(StringArray, "Pause", self._pause_cb, callback_group=service_server_cb_group)
        self.create_service(StringArray, "Stop", self._stop_cb, callback_group=service_server_cb_group)


    def _load_params(self):
        self.declare_parameter("map_path","/home/ros2_workspace/src_/traffic_planner/traffic_planer_data/switch_pixel_coords.yaml")
        self.declare_parameter("travel_plans_path","/home/ros2_workspace/src_/traffic_planner/traffic_planer_data/travel_plan.yaml")
        self.declare_parameter("tracker_topic","tracked_objects")


    def _parse_map(self, map:str) -> None:
        new_map = dict()
        map_dict = yaml.safe_load(map)
        
        # check if yaml is valid
        try:
            map_schema.validate(map_dict)
            self.get_logger().info("Map loaded and valid")
        except SchemaError as se:
            raise se

        # create nodes
        for node in map_dict["nodes"]:
            name = node["node_name"]
            trigger_radius = node["trigger_radius"]
            pos = np.array([
                    node["position"]["x"],
                    node["position"]["y"]
                ]),

            new_node = MapNode( name, pos, trigger_radius, list())
            new_map[name] = new_node
        
        # add connections
        for node in map_dict["nodes"]: 
            name = node["node_name"]
            for con in node["connections"]:
                new_map[name].connections.append(new_map[con])

        return new_map
    

    def _parse_travel_plan(self, travel_plans:str, raise_error_if_invalid:bool=False) -> int:
        travel_lan_dict = yaml.safe_load(travel_plans)
        status = 0
        # check if yaml is valid
        try:
            travel_plan_schema.validate(travel_lan_dict)
        except SchemaError as se:
            if raise_error_if_invalid:
                raise se
            else:
                self.get_logger().error(f"Travelplan has invalid formatt:\n {se}")
                status = 1
        
        # check if checkpoints wor with loaded map
        if self._travel_plan_feasible(travel_lan_dict):

            for plan in travel_lan_dict["travel_plans"]:
                name = plan["name"]
                if name in travel_lan_dict.keys():
                    self.get_logger().warning(f"overwriting travel plan {plan['name']} with new input")
                    status = 3

                # create checkpoints
                checkpoints = [Checkpoint(self._map[cp["node_name"]],cp["actions"]) for cp in plan["checkpoints"]]
                # create travel plan
                new_plan = TravelPlan(
                    name,
                    plan["loop_plan"],
                    checkpoints
                )
                self._travel_plans[name] = new_plan
                self.get_logger().info("Travel plan loaded and valid")
            else:
                status = 2
    
        
        return status
    
    def _travel_plan_feasible(self, travel_plan_dict:dict) ->bool:
        for travel_plan in travel_plan_dict["travel_plans"]:
            checkpoints = travel_plan["checkpoints"]
            for index, checkpoint in enumerate( checkpoints):
                node_name = checkpoint["node_name"]
                # check if checkpoint in map
                if node_name not in self._map.keys():
                    self.get_logger().error(f"Travel plan invalid: '{node_name}' not contained in map")
                    return False
                # check connection with next checkpoint 
                next_index = (index+1) % len( checkpoints)
                next_checkpoint_name = checkpoints[next_index]["node_name"]
                connections = self._map[node_name].connections
                is_connected = any( (node.name == next_checkpoint_name) for node in connections)
                if not is_connected:
                    self.get_logger().error(f"Travel plan invalid: ' no connection between '{node_name}' and {next_checkpoint_name}")
                    return False
        
        return True

    

    def _tracker_cb(self,msg:TrackedObjects) -> None:
        vehicles = msg.objects

        for vehicle in vehicles:
            
            vehicle_pos = np.array( [ vehicle.position.x, vehicle.position.y])
            for active_plan in self._active_plans.values():
                
                current_checkpoint = active_plan.travel_plan.checkpoints[active_plan.next_checkpoint]
                checkpoint_pos = current_checkpoint.map_node.position
                delta = np.linalg.norm( checkpoint_pos-vehicle_pos)
                
                if delta <= current_checkpoint.map_node.trigger_radius:
                    
                    self.get_logger().info(f"tinycar <{active_plan.tiny_car_address}> passed checkpoint <{active_plan.next_checkpoint}>")
                    self._execute_next_actions(active_plan)
                    
                    if active_plan.next_checkpoint == len(active_plan.travel_plan.checkpoints):
                        active_plan.next_checkpoint = 0
                        self.get_logger().info(f"tinycar <{active_plan.tiny_car_address}> finished travel")
                        
                        if not active_plan.travel_plan.loop_plan:
                            req = SetUint32KeyValue.Request()
                            req.key = active_plan.tiny_car_address
                            req.value = 0
                            self._tinycar_services.speed_service.call_async(req)

            
    def _add_travel_plan_cb(self, request, response):
        plan_str = request.value
        response.status = self._parse_travel_plan(plan_str)

        return response
    
    # TODO: Check if tinycar address is valid!
    def _assigne_travel_plan_cb(self, request, response):
        tracker_id = request.tracker_id
        ipaddress = request.ip_address
        plan_name = request.plan_name

        if plan_name in self._travel_plans.keys():

            plan = self._travel_plans[plan_name]
            new_travel_plan = ActiveTravelPlan(tracker_id, ipaddress, 0, plan, False)
            
            if ipaddress in self._active_plans.keys():
                req = SetUint32KeyValue.Request()
                req.key = ipaddress
                req.value = 0
                self._tinycar_services.speed_service.call_async(req)
                self.get_logger().warning(f"reassigning travel plan to vehicle with ip: {ipaddress} and tracking_id: {tracker_id}")
                response.status = 2
            else:
                response.status = 0
                self.get_logger().info(f"assigning travel plan to vehicle with ip: {ipaddress} and tracking_id: {tracker_id}")

            self._active_plans[ipaddress] = new_travel_plan

        else:
            response.status = 1

        return response
    

    def _start_cb(self, request, response):
        tinycar_addresses = request.strings
        response.status = 0

        for address in tinycar_addresses:
            if address not in self._active_plans.keys():
                response.status = 1
                continue
            
            activeTravelPlan = self._active_plans[address]
            if activeTravelPlan.pause:
                activeTravelPlan.next_checkpoint -=1      
                activeTravelPlan.pause = False

            self.get_logger().info(f"start travel plan for tinycar: {address}")
            self._execute_next_actions(activeTravelPlan)
            

        return response       
    

    def _pause_cb(self, request, response):
        tinycar_addresses = request.strings
        response.status = 0

        for address in tinycar_addresses:
            if address not in self._active_plans.keys():
                response.status = 1
                continue
            
            activeTravelPlan = self._active_plans[address]
            activeTravelPlan.pause = True

            req = SetUint32KeyValue.Request()
            req.key = activeTravelPlan.tiny_car_address
            req.value = 0
            self._tinycar_services.speed_service.call_async(req) 
            self.get_logger().info(f"pause travel plan for tinycar: {address}")

        return response
    

    def _stop_cb(self, request, response):
        tinycar_addresses = request.strings
        response.status = 0

        for address in tinycar_addresses:
            if address not in self._active_plans.keys():
                response.status = 1
                continue
            
            activeTravelPlan = self._active_plans[address]
            activeTravelPlan.pause = True

            req = SetUint32KeyValue.Request()
            req.key = activeTravelPlan.tiny_car_address
            req.value = 0
            self._tinycar_services.speed_service.call_async(req)

            activeTravelPlan.next_checkpoint = 0
            activeTravelPlan.pause = False
            self.get_logger().info(f"stop travel plan for tinycar: {address}")


    def _execute_next_actions(self, active_plan:ActiveTravelPlan) -> None:

        checkpoint = active_plan.travel_plan.checkpoints[active_plan.next_checkpoint]
        for action, value in checkpoint.actions.items():
            
            if action == "set_switch_state":
                req = SetState.Request()
                req.switch_name = checkpoint.map_node.name
                req.state_name = value
                self._switch_client.call(req)
                self.get_logger().info(f"set_switch_state: {checkpoint.map_node.name} -> {value}")

            if action == "set_speed":
                req = SetUint32KeyValue.Request()
                req.key = active_plan.tiny_car_address
                req.value = value
                self._tinycar_services.speed_service.call(req)
                self.get_logger().info(f"set_speed: address= {active_plan.tiny_car_address}, speed= {value}")

            if action == "set_headlight":
                req = SetUint32KeyValue.Request()
                req.key = active_plan.tiny_car_address
                req.value = HeadlightStates._member_map_[value].value
                self._tinycar_services.headlight_service.call(req)
                self.get_logger().info(f"set_headlight: address= {active_plan.tiny_car_address}, state= {HeadlightStates._member_map_[value]}")

            if action == "set_taillight":
                req = SetUint32KeyValue.Request()
                req.key = active_plan.tiny_car_address
                req.value = TaillightStates._member_map_[value].value
                self._tinycar_services.taillight_service.call(req)
                self.get_logger().info(f"set_taillight: address= {active_plan.tiny_car_address}, state= {TaillightStates._member_map_[value]}")

            if action == "set_blinker":
                req = SetUint32KeyValue.Request()
                req.key = active_plan.tiny_car_address
                req.value = BlinkerStates._member_map_[value].value
                self._tinycar_services.blinker_service.call(req)
                self.get_logger().info(f"set_blinker: address= {active_plan.tiny_car_address}, state= {BlinkerStates._member_map_[value]}")
        
        active_plan.next_checkpoint +=1
    

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    minimal_publisher = Planner()
    executor.add_node(minimal_publisher)

    executor.spin()

    minimal_publisher.destroy_node()
    executor.shutdown()


if __name__ == '__main__':
    main()