from kalman_filter import CVD_KalmanFilter
import numpy as np
from matplotlib import pyplot


true_initial_position = -20
true_velocity = 1
true_accelaration = 0

initial_state = np.array([
    0., # position
    1., # velocity
    0.  # accelaration
    ])

measurment_uncertainty = np.array([
    1 # std in measurment
])

processnoise = np.array([
    0.,   # position noise
    0.,   # velocity noise
    0.25   # accelaration noise
])

estimatation_uncertainty = np.array([
    0.01, # error in initial position estimate
    0.01, # error in initial velocity estimate
    0.01  # error in initial accelaration estimate
])

time_step = 0.1
update_rate = 5
iterations = 100
ground_truth = [(true_initial_position +true_velocity*(x*time_step) + 0.5*(x*time_step)**2*true_accelaration)  for x in range(iterations)]
estimates = np.zeros(iterations)
variances = np.zeros(iterations)


filter = CVD_KalmanFilter(dim_x=3, dim_z=1,initial_state=initial_state)

for i in range(iterations):
    filter.predict(time_step)
    if i % update_rate == 0:
        filter.update(ground_truth[i]+ np.random.rand()*measurment_uncertainty)

    estimates[i] = filter._H.dot(filter._X)
    variances[i] = filter._P[0][0]

pyplot.subplot(2,1,1)
pyplot.plot(ground_truth,'o-k')
pyplot.plot(estimates,'o-b')
pyplot.plot([ground_truth+ var for var in variances] ,'--m')
pyplot.title("Position")
pyplot.xlabel("time in ms")
pyplot.ylabel("range in meters")


pyplot.show()