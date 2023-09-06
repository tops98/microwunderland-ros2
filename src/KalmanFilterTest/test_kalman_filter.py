import unittest
from kalman_filter import CVD_KalmanFilter
import numpy as np


class TestCVDKalmanFilter(unittest.TestCase):
                                                    
    def test_dimensions_of_default_generated_matrices(self):
        state = np.array([1,2,3,4,5,6])
        num_states = 6
        num_measurments = 2
        filter = CVD_KalmanFilter(dim_x=num_states, dim_z=num_measurments, initial_state=state)

        self.assertEqual(filter._P.shape, (num_states,num_states))
        self.assertEqual(filter._R.shape, (num_measurments,num_measurments))
        self.assertEqual(filter._Q.shape,(num_states,num_states))
        self.assertEqual(filter._H.shape, (num_measurments, num_states))

    def test_error_for_wrong_num_of_states(self):
        state = np.array([1,2,4,5,6])
        num_states = 6
        num_measurments = 2
        self.assertRaises(ValueError,CVD_KalmanFilter, num_states, num_measurments, state)

    def test_if_uncertainty_increases_when_calling_predict(self):
        num_states = 6
        num_measurments = 2
        filter = CVD_KalmanFilter(dim_x=num_states, dim_z=num_measurments)
        for i in range(10):
            old_P = np.linalg.det(filter._P)
            filter.predict(dt=1)
            new_P = np.linalg.det(filter._P)

            self.assertGreater(new_P,old_P)

    def test_motion_equations(self):
        num_states = 6
        num_measurments = 2
        initial_state = np.array([3.5,11, 2,1, 0.7,0])
        filter = CVD_KalmanFilter(dim_x=num_states, dim_z=num_measurments, initial_state=initial_state)

        # s = pos + velocity * delta_time + 0.5 * delta_time**2 *accelaration
        expected_x_pos = 3.5 + 10*2 + 0.5* 10**2 * 0.7
        expected_y_pos = 11 + 10*1 + 0.5* 10**2 * 0
        # v = velocity + velocity * delta_time
        expected_x_vel = 2 + 10 * 0.7
        expected_y_vel = 1 + 10 * 0

        for i in range(10):
            filter.predict(dt=1)
            new_P = np.linalg.det(filter._P)
        
        self.assertAlmostEqual(expected_x_pos,filter._X[0])
        self.assertAlmostEqual(expected_y_pos,filter._X[1])

        self.assertAlmostEqual(expected_x_vel,filter._X[2])
        self.assertAlmostEqual(expected_y_vel,filter._X[3])

    
    def test_if_uncertainty_decreases_when_calling_update(self):
        num_states = 6
        num_measurments = 2

        x_y_measurments = np.array([5,22.5])

        filter = CVD_KalmanFilter(dim_x=num_states, dim_z=num_measurments)

        for i in range(10):
            old_P = np.linalg.det(filter._P)
            filter.update(x_y_measurments + np.array([1,1]) * np.random.rand())
            new_P = np.linalg.det(filter._P)
            print(new_P)

            self.assertGreater(old_P,new_P)
       


if __name__ == "__main__":
    unittest.main()