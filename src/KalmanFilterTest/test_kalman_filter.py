import unittest
from kalman_filter import CVD_KalmanFilter, CVD_KF_Config
import numpy as np


class TestCVDKalmanFilter(unittest.TestCase):


    def test_error_for_wrong_num_of_states(self):
        state = np.array([1,2,4,5,6])
        config = self._get_default_config()
        self.assertRaises(ValueError,CVD_KalmanFilter, config, state)

    def test_if_uncertainty_increases_when_calling_predict(self):
        state = np.zeros(6)
        config = self._get_default_config()
        filter = CVD_KalmanFilter(config,state)
        
        old_p = filter._P.diagonal()
        for i in range(10):
            filter.predict(1)
            new_p = filter._P.diagonal()
            for id in range(state.size):
                self.assertGreater(new_p[id], old_p[id])
            old_p = new_p
    
    def test_if_uncertainty_decreases_when_calling_update(self):
        measurments = np.array([2,0])
        state = np.array([2,53,0,0,0,0])
        config = self._get_default_config()

        filter = CVD_KalmanFilter(config, state)
        old_p = filter._P.diagonal()

        for i in range(10):
            filter.update(measurments)
            new_p = filter._P.diagonal()

            for id in range(state.size):
                self.assertLess(new_p[id], old_p[id])
            old_p = new_p

    def test_motion_equations(self):
        config = self._get_default_config()
        initial_state = np.array([3.5,11, 2,1, 0.7,0])
        filter = CVD_KalmanFilter(config, initial_state=initial_state)

        # s = pos + velocity * delta_time + 0.5 * delta_time**2 *accelaration
        expected_x_pos = 3.5 + 10*2 + 0.5* 10**2 * 0.7
        expected_y_pos = 11 + 10*1 + 0.5* 10**2 * 0
        # v = velocity + velocity * delta_time
        expected_x_vel = 2 + 10 * 0.7
        expected_y_vel = 1 + 10 * 0

        for i in range(10):
            filter.predict(dt=1)
        
        self.assertAlmostEqual(expected_x_pos,filter._X[0])
        self.assertAlmostEqual(expected_y_pos,filter._X[1])

        self.assertAlmostEqual(expected_x_vel,filter._X[2])
        self.assertAlmostEqual(expected_y_vel,filter._X[3])

    def _get_default_config(self) -> CVD_KF_Config:
        config = CVD_KF_Config(
            num_measurements= 2,
            initial_uncertenty= np.array([1,1,1,1,1,1]),
            measurment_uncertenty= np.array([1,1]),
            process_noise= np.array([1,1,1,1,1,1]))
        return config
       


if __name__ == "__main__":
    unittest.main()

