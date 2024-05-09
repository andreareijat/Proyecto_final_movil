import numpy as np
from Simulator import Landmark, Map, Robot, EnvPlot
from scipy.stats import chi2

noLandmarks = 17
m = Map()
for i in range(noLandmarks):
    p = np.array([np.random.uniform(-5,5), np.random.uniform(-5,5)], np.float32)
    m.append(Landmark(p, i))

e = EnvPlot()

# Set the state and measurement noise covariance
Q = np.array([[0.25**2, 0],[0, 0.25**2]], np.float32) #covarianza del ruido de estado (ROBOT)
R = np.array([[0.2**2, 0],[0, 0.2**2]], np.float32) #covarianza del ruido de medida (LANDMARKS)

# Create the robot object, by default measures the Cartesian coordinates of the landmarks in the environment
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
        self.H = None #Jacobiana
        self.S = None #Covarianza
        pass


    def add_landmarks(self,l):
        pass
        

    """
    Cada observación real yk+1 se compara con la observación predicha y^k+1
    para determinar a qué landmark conocido podría corresponder cada observación. 
    """
    def association(self, measurements, predicted_observations): 

        threshold = chi2.ppf(0.99, df=2) 
        associations = [-1] * len(measurements)
        

        for i, measurement in enumerate(measurements): 
            dm_squared = float('inf')
            
            for j, predicted_observation in enumerate(predicted_observations):
                # d_squared = (measurement.p-predicted_observation).T@np.linalg.inv(jacobiana@self.Pk_k@jacobiana.T + self.R)@(measurement.p-predicted_observation)
                d_squared = np.linalg.norm(measurement - predicted_observation)
             
                #Si la distancia es menor a la minima actual
                if d_squared < dm_squared and d_squared < threshold:
                    dm_squared = d_squared
                    jm = j
                    associations[i] = j

        return associations
    


    """
    Calcula las observaciones predichas de todos los landmarks a partir del estado
    predicho del robot.
    landmarks = [[lx1, ly1],[lx2, ly2],...]
    """
    def predict_observations(self):
        
        predicted_observations = []

        robot_position = self.xk_k[:2]

        for l in self.map:
            d = l - robot_position
            predicted_observations.append(d)

        return predicted_observations



    """
    xk+1
    Predice solo la posición del robot y su covarianza.
    No afecta a los landmarks
    """
    def predict(self, u):
            self.xk_k[:2] = self.A[:2,:2]@self.xk_k[:2] + self.B[:2,:2]@u
            self.Pk_k = self.A[:2,:2]@self.Pk_k[:2,:2]@self.A[:2,:2].T + self.Q



    """
    Actualiza la posicion del robot, la de los landmarks
    y sus respectivas covarianzas basándose en la ganancia
    de Kalman
    """
    def update(self, y, a): 

        for idx, observation in enumerate(y): 

            #Si hay una asociacion valida
            if a[idx] != -1: 
                
                predicted_observation = self.predict_observations()[idx]
                innovation = observation - predicted_observation
        
                # Jacobiana de la observación
                H = np.tile([[-1, 0], [0, -1]], (1, 1))  
                I = np.eye(2)
                
                # Cálculo de la matriz S
                S = H @ self.Pk_k @ H.T + self.R

                # Ganancia de Kalman
                K = self.Pk_k @ H.T @ np.linalg.inv(S)

                # Actualización del estado y la covarianza
                self.xk_k[:2] = self.xk_k[:2] + (K @ innovation)
                self.Pk_k = self.Pk_k - K@S@K.T

            else:
                self.add_landmarks(observation)



# Initial estimates of the position of the error and its covariance matrix
xHat = np.array([0, 0], np.float32)
PHat = np.array([[3,0],[0,3]], np.float32) 

kf = KF(xHat, PHat, 0.01*Q,  R, m)

e.plotSim(r, m, kf, True)

#estoy en Xk|k

while r.t < r.tf:

    u = r.action()
    kf.predict(u)

    measurements = r.measure(m) 
    predicted_observations = kf.predict_observations()

    #comprobar en base a que sistema de referencia se hacen las dos cosas
    a = kf.association(measurements, predicted_observations)

    kf.update(measurements,a) 

    e.plotSim(r, m, kf)


"""
Ahora mismo realiza las asociaciones pero no ejecuta correctamente el update.
Esta sin implementar la funcion de add_landmarks en caso de no existir asociación.

El motivo principal es la duda que tengo, 
El robot debería de tener un mapa interno que se inicialice a cero? De esta forma las 
predicted_landmarks se hacen solo de la posición predicha del robot a cada una de las
landmarks del mapa interno del robot?

O se deben emplear igualmente todos los landmarks del mapa completo? Es que así el robot
ya "conocería" todo el mapa y no iria creando uno nuevo no?
"""


