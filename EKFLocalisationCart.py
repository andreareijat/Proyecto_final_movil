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
R = np.array([[0.2**2, 0],[0, 0.2**2]], np.float32)

# Create the robot object, by default measures the Cartesian
# coordinates of the landmarks in the environment
r = Robot(np.array([2,-2], np.float32), Q, R)

class KF:
    def __init__(self, x0, P0, Q, R, m, dt = r.dt):
        self.dt = dt
        self.xk_k = x0
        self.Pk_k = P0
        self.Q = Q * self.dt
        self.R = R / self.dt
        self.type = type
        self.map = m
        self.A = np.eye(len(self.xk_k))
        self.B = self.dt * np.eye(len(self.xk_k))
        pass

    
    """
    Predice solo la posición del robot y su covarianza.
    No afecta a los landmarks.
    """
    def predict(self, u):
            self.xk_k[:2] = self.A[:2,:2]@self.xk_k[:2] + self.B[:2,:2]@u
            self.Pk_k = self.A[:2,:2]@self.Pk_k[:2,:2]@self.A[:2,:2].T + self.Q
    

    def update(self, y):
        pass


    """
    Agrega nuevos landmarks al mapa basándose en las mediciones
    actuales. Los añade con el mismo id del landmark detectado.
    """
    def add_landmarks(self,measurements):

        for measure in measurements: 

            global_position = self.transform_to_global(measure)
            new_landmark = Landmark(global_position, measure.id)
            self.xk_k = np.append(self.xk_k, new_landmark)    
    
    """
    Transforma una posición relativa a una global basada en la posición 
    del robot
    """
    def transform_to_global(self, measure): 
        
        x,y = self.xk_k[:2]
        dx, dy = measure.p
        
        global_x = x + dx
        global_y = y + dy

        return np.array([global_x, global_y], dtype=np.float32)
    

    """
    """
    def do_association(self, measurements):
        pass
        

# Initial estimates of the position of the error and its covariance matrix
xHat = np.array([0, 0], np.float32)
PHat = np.array([[3,0],[0,3]], np.float32)

# Object for the (Extended) Kalman filter estimator
kf = KF(xHat, PHat, Q,  R, m)

# Plot the first step of the estimation process and pause
e.plotSim(r, m, kf, True)

measurements = r.measure(m)
kf.add_landmarks(measurements)

while r.t < r.tf:
    # print("Measurements: ", measurements)
    print("Estado: ", kf.xk_k)
    u = r.action()
    kf.predict(u)
    measurements = r.measure(m)
    associations = kf.do_association(measurements)
#     kf.update(y)

    kf.add_landmarks(measurements)
    
    e.plotSim(r, m, kf)
