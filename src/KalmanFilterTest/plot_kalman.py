from kalman_filter import CVD_KalmanFilter
import numpy as np
from matplotlib import pyplot


true_initial_position = 3
true_velocity = 3
true_accelaration = 0

initial_state = np.array([
    0., # position
    1., # velocity
    0.  # accelaration
    ])

measurment_uncertainty = np.array([
    3 # std in measurment
])

processnoise = np.array([
    0.045,   # position noise
    0.045,   # velocity noise
    0.045   # accelaration noise
])

estimatation_uncertainty = np.array([
    1,  # error in initial position estimate
    5,   # error in initial velocity estimate
    10   # error in initial accelaration estimate
])

time_step = 0.1
update_rate = 10
iterations = 300

ground_truth = [(true_initial_position +true_velocity*(x*time_step) + 0.5*(x*time_step)**2*true_accelaration)  for x in range(iterations)]
measurments = np.zeros(iterations)
pos_estimates = np.zeros(iterations)
vel_estimates = np.zeros(iterations)
acc_estimates = np.zeros(iterations)
variances = np.zeros((3,iterations))
error = 0



for j in range(2000):
    filter = CVD_KalmanFilter(dim_x=3, dim_z=1,initial_state=initial_state)
    filter._P = estimatation_uncertainty.dot(estimatation_uncertainty.transpose())
    filter._R = measurment_uncertainty.dot(measurment_uncertainty.transpose())
    filter._Q = processnoise.dot(processnoise.transpose())
    for i in range(iterations):
        measurment = ground_truth[i]+ np.random.uniform(-1, 1)*measurment_uncertainty
        filter.predict(time_step)
        if not (i >100 and i < 200):
            if i % update_rate == 0:
                filter.update(measurments=measurment)

        measurments[i] = measurment
        pos_estimates[i] = filter._H.dot(filter._X)
        variances[:,i] = filter._P.diagonal()

        vel_estimates[i] = filter._X[1]
        acc_estimates[i] = filter._X[2]
        error += abs(filter._X[0] - ground_truth[i])

pyplot.subplot(3,1,1)
pyplot.plot(ground_truth,'-k')
pyplot.plot(measurments,'o-b',markersize=4)
pyplot.plot(pos_estimates,'o-g',markersize=4)
pyplot.plot([(ground_truth[i] + 2*np.sqrt(abs(variances[0][i])) ,ground_truth[i] - 2*np.sqrt(abs(variances[0][i]))) for i in range(iterations)] ,'--m')

pyplot.title("Position")
pyplot.xlabel("time in ms")
pyplot.ylabel("range in meters")
pyplot.legend(["true val", "measurment", "estimate", "variance"],loc='center left', bbox_to_anchor=(1, 0.5))

pyplot.subplot(3,1,2)
pyplot.axhline(y = true_velocity, color = 'k', linestyle = '-')
pyplot.plot(vel_estimates,"o-r",markersize=4)
pyplot.plot([(true_velocity + 2*np.sqrt(abs(variances[1][i])) ,true_velocity - 2*np.sqrt(abs(variances[1][i]))) for i in range(iterations)] ,'--m')
pyplot.title("Velocity")
pyplot.xlabel("time in ms")
pyplot.ylabel("m/s")
pyplot.legend(["true velocity", "estimate", "variance"],loc='center left', bbox_to_anchor=(1, 0.5))

pyplot.subplot(3,1,3)
pyplot.axhline(y = true_accelaration, color = 'k', linestyle = '-')
pyplot.plot(acc_estimates,"o-r",markersize=4)
pyplot.plot([(true_accelaration + 2*np.sqrt(abs(variances[2][i])) ,true_accelaration - 2*np.sqrt(abs(variances[2][i]))) for i in range(iterations)] ,'--m')
pyplot.title("Accelaration")
pyplot.xlabel("time in ms")
pyplot.ylabel("m/s")
pyplot.legend(["true accelaration", "estimate", "variance"],loc='center left', bbox_to_anchor=(1, 0.5))

pyplot.tight_layout(h_pad=0.5)
print(error/(iterations*2000))
pyplot.show()