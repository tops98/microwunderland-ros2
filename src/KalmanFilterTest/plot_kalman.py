from kalman_filter import CVD_KalmanFilter, CVD_KF_Config
import numpy as np
from matplotlib import pyplot


true_initial_position = 3
true_velocity = 3
true_accelaration = 0

initial_state = np.array([
    -3.,  # position
    0., # velocity
    2.   # accelaration
    ])

measurment_uncertainty = np.array([
    3 # std in measurment
])

processnoise = np.array([
    0.9,   # position noise
    0.45,   # velocity noise
    0.1   # accelaration noise
])

estimatation_uncertainty = np.array([
    100,  # error in initial position estimate
    1,   # error in initial velocity estimate
    1   # error in initial accelaration estimate
])

time_step = 0.1
update_rate = 1
iterations = 1000
repetitions = 1

# ground_truth = [(true_initial_position +true_velocity*(x*time_step) + 0.5*(x*time_step)**2*true_accelaration)  for x in range(iterations)]
ground_truth = np.zeros(iterations)
for x in range(iterations):
    if x == 600:
        true_velocity +=3
    ground_truth[x] = true_initial_position +true_velocity*(x*time_step) + 0.5*(x*time_step)**2*true_accelaration

measurments = list()
pos_estimates = np.zeros(iterations)
vel_estimates = np.zeros(iterations)
acc_estimates = np.zeros(iterations)
uncertainty = np.zeros(iterations)
variances = np.zeros((3,iterations))
error = 0

for j in range(repetitions):
    rng = np.random.default_rng(seed=j)
    measurments = list()

    filter_config = CVD_KF_Config(
        num_measurements=1,
        initial_uncertenty=estimatation_uncertainty,
        measurment_uncertenty=measurment_uncertainty,
        process_noise=processnoise
    )
    filter = CVD_KalmanFilter(filter_config,initial_state=initial_state)
    
    for i in range(iterations):
        filter.predict(time_step)
        if not (i >300 and i <500):
            if i % update_rate == 0:
                measurment = ground_truth[i]+rng.standard_normal()*measurment_uncertainty
                filter.update(measurments=measurment)
                measurments.append((i,measurment))

        pos_estimates[i] = filter._H.dot(filter._X)
        variances[:,i] = filter._P.diagonal()

        vel_estimates[i] = filter._X[1]
        acc_estimates[i] = filter._X[2]
        uncertainty[i] = abs(np.linalg.det(filter._P))
        error += abs(filter._X[0] - ground_truth[i])
        # error += abs(measurment - ground_truth[i])

# pyplot.subplot(4,1,1)
pyplot.plot(ground_truth,'-k')
pyplot.plot( [x[0] for x in measurments], [y[1] for y in measurments],'o-b',markersize=4)
pyplot.plot(pos_estimates,'o-g',markersize=4)
pyplot.plot([(ground_truth[i] + 2*np.sqrt(abs(variances[0][i])) ,ground_truth[i] - 2*np.sqrt(abs(variances[0][i]))) for i in range(iterations)] ,'--m')

pyplot.title("Position")
pyplot.xlabel("time in ms")
pyplot.ylabel("range in meters")
pyplot.legend(["true val", "measurment", "estimate", "variance"],loc='center left', bbox_to_anchor=(1, 0.5))

# pyplot.subplot(4,1,2)
# pyplot.axhline(y = true_velocity, color = 'k', linestyle = '-')
# pyplot.plot(vel_estimates,"-r",markersize=4)
# pyplot.plot([(true_velocity + 2*np.sqrt(abs(variances[1][i])) ,true_velocity - 2*np.sqrt(abs(variances[1][i]))) for i in range(iterations)] ,'--m')
# pyplot.title("Velocity")
# pyplot.xlabel("time in 1/10s")
# pyplot.ylabel("m/s")
# pyplot.legend(["true velocity", "estimate", "variance"],loc='center left', bbox_to_anchor=(1, 0.5))

# pyplot.subplot(4,1,3)
# pyplot.axhline(y = true_accelaration, color = 'k', linestyle = '-')
# pyplot.plot(acc_estimates,"-r",markersize=4)
# pyplot.plot([(true_accelaration + 2*np.sqrt(abs(variances[2][i])) ,true_accelaration - 2*np.sqrt(abs(variances[2][i]))) for i in range(iterations)] ,'--m')
# pyplot.title("Accelaration")
# pyplot.xlabel("time in 1/10s")
# pyplot.ylabel("m/s")
# pyplot.legend(["true accelaration", "estimate", "variance"],loc='center left', bbox_to_anchor=(1, 0.5))

# pyplot.subplot(4,1,4)
# pyplot.plot(uncertainty,'-r')
# pyplot.title("Uncertainty in System")

# pyplot.tight_layout(h_pad=0.25)
print(f"Error: {error/(iterations*repetitions)}\nX= {filter._X}")
pyplot.show()