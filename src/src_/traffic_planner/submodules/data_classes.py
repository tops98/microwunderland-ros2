from dataclasses import dataclass
from typing import List, Dict
from enum import Enum
import numpy as np
from rclpy.node import Client


class BlinkerStates (Enum):
    OFF = 0
    LEFT = 1
    RIGHT = 2
    HAZARD = 3
class TaillightStates (Enum):
    ON = 0
    OFF = 1
    BRAKE = 2
class HeadlightStates (Enum):
    OFF = 0
    ON = 1

@dataclass
class TinycarService:
    battery_service:Client
    speed_service:Client
    headlight_service:Client
    taillight_service:Client
    blinker_service:Client


@dataclass
class MapNode:
    name:str
    position:np.ndarray
    trigger_radius:float
    connections:list

@dataclass 
class Checkpoint:
    map_node: MapNode
    actions:Dict[str, any]

@dataclass
class TravelPlan:
    name:str
    loop_plan:bool
    checkpoints:List[Checkpoint]

@dataclass
class ActiveTravelPlan:
    tracker_id:int
    tiny_car_address:str
    next_checkpoint:int
    travel_plan:TravelPlan
    pause:bool