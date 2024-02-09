import numpy as np
from dataclasses import dataclass
from typing import List
from object_tracker.submodules.kalman_filter import CVD_KalmanFilter, CVD_KF_Config
from datetime import datetime,timedelta


class Centroid:
    _id: int
    _filter: CVD_KalmanFilter 
    _num_observed_elements:int
    _last_update:datetime
    unmatched:bool
    number_of_updates: int


    def __init__(self, id:int, filter:CVD_KalmanFilter) -> None:
        self._id = id
        self._filter = filter
        self._num_observed_elements = filter._R.shape[0]
        self.unmatched = False
        self.number_of_updates = 0
        self._last_update = datetime.now()

    def update(self, position:np.ndarray) -> None:
        self._filter.update(position)
        self._last_update = datetime.now()
        self.number_of_updates +=1
    
    def predict(self, delta_time:float) -> None:
        self._filter.predict(dt=delta_time)

    @property
    def position(self) -> np.ndarray:
        return self._filter._X[0:self._num_observed_elements]
    
    @property
    def velocity(self) -> np.ndarray:
        return self._filter._X[self._num_observed_elements:self._num_observed_elements*2]
    
    @property
    def acceleration(self) -> np.ndarray:
        return self._filter._X[self._num_observed_elements*2:self._num_observed_elements*3]
    
    @property
    def last_update(self) -> datetime:
        return self._last_update
    @property
    def id(self) -> int:
        return self._id


class CentroidTracker:
    _delta_max:float
    _max_update_pause_ms:float
    _centroids:List[Centroid]
    _id_count:int
    _filter_config: CVD_KF_Config
    _num_probation_updates: int

    def __init__(self, delta_max:float, max_update_pause_ms:float, filter_config: CVD_KF_Config, num_probation_updates: int = 5) -> None:
        self._delta_max = delta_max
        self._max_update_pause_ms = max_update_pause_ms
        self._centroids = list()
        self._id_count = 0
        self._num_probation_updates = num_probation_updates
        self._filter_config =filter_config

    def update(self, positons:np.ndarray) -> None:
        # check for match with existing centroids
        positons = self._find_matching_centroid(positons)
        # register new centroids
        self._register(positons)
        # remove lost objects
        self._remove_lost_objects()
    
    def _find_matching_centroid(self, positions:np.ndarray) -> np.ndarray:
        # set all centroids to not updated
        for centroid in self._centroids: centroid.unmatched = True
        new_positions = list()
        
        for pos in positions[:]:
            
            min_dist = self._delta_max
            closest_centroid = None
            
            # find closest centroid
            for centroid in self._centroids:
                current_dist = np.linalg.norm(centroid.position-pos)
                if current_dist < min_dist:
                    min_dist = current_dist
                    closest_centroid = centroid                  
            
            # if centroid within delta max found update centroid with new pos
            if closest_centroid is not None:
                closest_centroid.update(pos)
                closest_centroid.unmatched = False

            else:
                new_positions.append(pos)

        return np.array(new_positions)
    
    def _register(self, positions:np.ndarray) -> None:              
        for pos in positions:
            initial_state = np.zeros(pos.size*3)
            initial_state[0:pos.size] = pos
            filter = CVD_KalmanFilter(self._filter_config,initial_state)

            self._centroids.append( Centroid(self._id_count, filter))
            self._id_count+=1

    def _remove_lost_objects(self) -> None:
        for centroid in self._centroids[:]:
            delta_time = datetime.now() - centroid.last_update
            if delta_time >= timedelta(milliseconds=self._max_update_pause_ms):
                self._centroids.remove(centroid)

    @property
    def get_state(self) ->List[Centroid]:
        return self._centroids