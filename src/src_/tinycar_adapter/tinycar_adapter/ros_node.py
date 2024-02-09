import rclpy
from rclpy.node import Node
from typing import Dict
from submodules.tinycarUdpClient import TinycarUdpClient, HeadlightStates, TaillightStates, BlinkerStates
from microwunderland_interfaces.srv import GetFloat32, SetUint32KeyValue, GetStringArray


class TinycarAdapter(Node):
    _tinycar_clients: Dict[str,TinycarClient]


    def __init__(self):
        super().__init__('tiny_car_adapter')
        
        self._load_params()
        self._tinycar_clients = self._create_tinycar_clients()

        # create services
        self.create_service(GetFloat32,"get_battery_status", self._get_battery_status_cb)

        self.create_service(GetStringArray,"get_available_cars",self._get_available_cars_cb)
        self.create_service(SetUint32KeyValue,"set_speed",self._set_speed_cb)
        self.create_service(SetUint32KeyValue,"set_head_light",self._set_head_light_cb)
        self.create_service(SetUint32KeyValue,"set_tail_light",self._set_tail_light_cb)
        self.create_service(SetUint32KeyValue,"set_blinker",self._set_blinker_cb)

        self.get_logger().info(f"{self.get_name()} is ready")


    def _load_params(self) -> None:
        self.declare_parameter("tinycar_udp_port",55003)
        self.declare_parameter("planner_udp_port",55002)
        self.declare_parameter("tinycar_addresses", ["tinycar01.local"])


    def _create_tinycar_clients(self) -> Dict[str,TinycarClient]:
        tiny_car_clients = dict()
        car_port = self.get_parameter("tinycar_udp_port").get_parameter_value().integer_value
        listen_port = self.get_parameter("planner_udp_port").get_parameter_value().integer_value
        addresses = self.get_parameter("tinycar_addresses").get_parameter_value().string_array_value

        for address in addresses:
            new_client = TinycarClient(address, car_port, listen_port)
            tiny_car_clients[address] = new_client
        
        return tiny_car_clients

    def _get_available_cars_cb(self, request, response):
        response.value = self._tinycar_clients.keys()
        return response

    def _get_battery_status_cb(self, request, response):
        car_name = request.key
        response.status = 1
        if car_name in self._tinycar_clients:        
            response.value = self._tinycar_clients[car_name].get_battery_status()
            response.status = 0
        else:
            self.get_logger().error(f"'{car_name}' not in list of available car clients!")
        return response
    

    def _set_speed_cb(self, request, response):
        car_name = request.key
        response.status = 1
        if car_name in self._tinycar_clients: 
            speed = request.value
            self._tinycar_clients[car_name].setMotorSpeed(speed)
            response.status = 0
        else:
            self.get_logger().error(f"'{car_name}' not in list of available car clients!")
        
        return response
    
    
    def _set_head_light_cb(self, request, response):
        car_name = request.key
        response.status = 1
        
        if car_name in self._tinycar_clients: 
            mode = request.value
        
            if mode in HeadlightStates._value2member_map_:
                state = HeadlightStates._value2member_map_[mode]
                self._tinycar_clients[car_name].setHeadlightState(state)
                response.status = 0
            else:
                self.get_logger().error(f"no mapping for state'{mode}'")
        else:
           self.get_logger().error(f"'{car_name}' not in list of available car clients!")
        return response
    
    
    def _set_tail_light_cb(self, request, response):
        car_name = request.key
        response.status = 1
        
        if car_name in self._tinycar_clients: 
            mode = request.value
        
            if mode in TaillightStates._value2member_map_:
                state = TaillightStates._value2member_map_[mode]
                self._tinycar_clients[car_name].setTaillightState(state)
                response.status = 0
            else:
                self.get_logger().error(f"no mapping for state'{mode}'")
        else:
            self.get_logger().error(f"'{car_name}' not in list of available car clients!")

        return response
    
    
    def _set_blinker_cb(self, request, response):
        car_name = request.key
        response.status = 1
        
        if car_name in self._tinycar_clients: 
            mode = request.value
        
            if mode in BlinkerStates._value2member_map_:
                state = BlinkerStates._value2member_map_[mode]
                self._tinycar_clients[car_name].setBlinkertState(state)
                response.status = 0
            else:
                self.get_logger().error(f"no mapping for state'{mode}'")
        else:
            self.get_logger().error(f"'{car_name}' not in list of available car clients!")

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = TinycarAdapter()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()