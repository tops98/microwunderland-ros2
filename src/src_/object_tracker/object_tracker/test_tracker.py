import unittest
from submodules.kalman_filter import CVD_KalmanFilter, CVD_KF_Config
from submodules.centroid_tracker import CentroidTracker, Centroid
import numpy as np
from pprint import pprint
from datetime import datetime,timedelta


class TestTracker(unittest.TestCase):

    def test_registration(self):
        positions = np.array([
            [12,12],
            [256, 100],
            [22,42],
            [13,12]]
        )

        tracker = CentroidTracker(
            delta_max=5,
            max_update_pause_ms=3000,
            filter_config=self._get_default_filter_config()
        )

        tracker._register(positions)
        self.assertEqual(len(tracker._centroids), positions[:,0].size)

        for centroid in tracker._centroids:
            self.assertAlmostEqual(centroid.position[0], positions[centroid.id][0])
            self.assertAlmostEqual(centroid.position[1], positions[centroid.id][1])

    def test_removal(self):
        tracker = CentroidTracker(
            delta_max=5,
            max_update_pause_ms=3000,
            filter_config=self._get_default_filter_config()
        )

        centroids = [ Centroid(id, self._get_default_filter_config()) for id in range(5)]
        centroids[0]._last_update = datetime.now() - timedelta(milliseconds=3000)
        centroids[1]._last_update = datetime.now() - timedelta(milliseconds=2999)
        centroids[2]._last_update = datetime.now() - timedelta(milliseconds=1000)
        centroids[3]._last_update = datetime.now() - timedelta(milliseconds=1)
        centroids[4]._last_update = datetime.now()
        tracker._centroids = centroids

        tracker._remove_lost_objects()
        self.assertEqual(len(tracker._centroids), 4)

    def test_matching(self):

        config = CVD_KF_Config(
            num_measurements= 1,
            initial_uncertenty= np.array([1,0,0]),
            measurment_uncertenty= np.array([0]),
            process_noise= np.array([0.01,0.01,0.01])
        )
        tracker = CentroidTracker(
            delta_max=5,
            max_update_pause_ms=3000,
            filter_config=config
        )

        initial_positions = np.array([
            [10],
            [50],
            [100],
            [200],
        ])
        new_positions = np.array([
            [14.999],
            [55],
            [95.999],
            [200],
        ])
        
        tracker._register(initial_positions)
        tracker._find_matching_centroid(new_positions)
        
        centroids = tracker._centroids
        
        self.assertAlmostEqual(centroids[0].position[0],new_positions[0][0])
        self.assertAlmostEqual(centroids[1].position[0],initial_positions[1][0])
        self.assertAlmostEqual(centroids[2].position[0],new_positions[2][0])
        self.assertAlmostEqual(centroids[3].position[0],new_positions[3][0])

        self.assertTrue(not centroids[0].unmatched)
        self.assertTrue(    centroids[1].unmatched)
        self.assertTrue(not centroids[2].unmatched)
        self.assertTrue(not centroids[3].unmatched)


    def _get_default_filter_config(self) -> CVD_KF_Config:
        config = CVD_KF_Config(
            num_measurements= 2,
            initial_uncertenty= np.array([1,1,1,1,1,1]),
            measurment_uncertenty= np.array([1,1]),
            process_noise= np.array([1,1,1,1,1,1]))
        return config
    
if __name__ == "__main__":
    unittest.main()