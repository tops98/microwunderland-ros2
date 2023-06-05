from dataclasses import dataclass
from typing import List
import numpy.typing as npt

@dataclass
class MapNode:
    name:str
    position:npt.ArrayLike
    trigger_radius:float
    connections:List[str]

@dataclass
class Checkpoint:
    node:MapNode
    state:str

@dataclass
class TravelPlan:
    start_time: float
    loop: bool
    next_checkpoint: int
    checkpoints:List[Checkpoint]
    
@dataclass
class TrackedObject:
    name:str
    position:npt.ArrayLike
    travel_plan: TravelPlan
