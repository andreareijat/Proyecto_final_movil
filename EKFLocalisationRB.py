import numpy as np
from Simulator import Landmark, Map, Robot, EnvPlot

# Create a map with random landmarks
noLandmarks = 25
m = Map()
for i in range(noLandmarks):
    p = np.array([np.random.uniform(-5,5), np.random.uniform(-5,5)], np.float32)
    m.append(Landmark(p, i))

# Create an object to plot the environment (map), robot and estimate
e = EnvPlot()
    
# Set the state and measurement noise covariance
Q = np.array([[0.25**2, 0],[0, 0.25**2]], np.float32)
R = np.array([[0.2**2, 0],[0, (5 * np.pi / 180)**2]], np.float32)

# Create the robot object, by default measures the Cartesian
# coordinates of the landmarks in the environment
r = Robot(np.array([2,-2], np.float32), Q, R, type='rb', range=2.5)

class KF:
    def __init__(self, x0, P0, Q, R, m, dt = r.dt):
        self.dt = dt
        self.xk_k = x0
        self.Pk_k = P0
        self.Q = Q * self.dt
        self.R = R / self.dt
        self.type = type
        self.map = m
        pass
    
    def predict(self, u):
        # TODO: Implement the prediction step of the KF        
        pass
    def update(self, y):
        # TODO: Implement the updarte step of the KF for localization
        # using the field 'map' of the class
        pass
    
# Initial estimates of the position of the error and its covariance
# matrix
xHat = np.array([0, 0], np.float32)
PHat = np.array([[3,0],[0,3]], np.float32)

# Object for the (Extended) Kalman filter estimator
kf = KF(xHat, PHat, Q,  R, m)
        
# Plot the first step of the estimation process and pause
e.plotSim(r, m, kf, True)

# Main loop:
# 1- The robot moves (action) and returns its velocity
# 2- Predict step of the Kalman filter
# 3- The robot perceives the environment (landmarks) and
#    returns a list of landmarks
# 4- Update state of the Kalman filter
while r.t < r.tf:
    u = r.action()
    kf.predict(u)
    y = r.measure(m)
    kf.update(y)
    
    e.plotSim(r, m, kf)
