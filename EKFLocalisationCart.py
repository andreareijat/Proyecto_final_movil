import numpy as np
from Simulator import Landmark, Map, Robot, EnvPlot
import time
# Create a map with random landmarks
noLandmarks = 15
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
        self.landmark_ids = []
        pass

    
    """
    Predice solo la posición del robot y su covarianza.
    No afecta a los landmarks.
    """
    def predict(self, u):
            self.xk_k[:2] = self.A[:2,:2]@self.xk_k[:2] + self.B[:2,:2]@u
            self.Pk_k[:2,:2] = self.A[:2,:2]@self.Pk_k[:2,:2]@self.A[:2,:2].T + self.Q
            print("PK: ", kf.Pk_k)

    def update(self, y):
        pass


    """
    Agrega nuevos landmarks al mapa basándose en las mediciones
    actuales. Los añade con el mismo id del landmark detectado.
    """
    def add_landmarks(self,measurements, associations):

        for measure, assoc in zip(measurements, associations): 

            if assoc is None and (measure.id not in self.landmark_ids): 
                #se añade un nuevo landmark
                global_position = self.transform_to_global(measure)
                new_landmark = Landmark(global_position, measure.id)
                self.landmark_ids.append(measure.id)
                self.xk_k = np.append(self.xk_k, new_landmark)    

                #Añadir covarianza de las mediciones
                #la incertidumbre inicial de las landmarks debe ser inf
                #definimos un valor alto
                new_cov = np.eye(2) * 1000 #alta incertidumbre inicial
                self.Pk_k = np.block([
                    [self.Pk_k, np.zeros((self.Pk_k.shape[0], 2))],
                    [np.zeros((2, self.Pk_k.shape[1])), new_cov]
                ])
                
                # self.Pk_k=self.Pk_k
            else: 
                pass
                #TODO: se deberia actualizar el landmark existente con el ID asociado?
                # for landmark in self.xk_k:
                #     if landmark.id == assoc:
                #         landmark.update_position(measure.p)  # Función para actualizar la posición del landmark
                #         break
    
    
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
    Asocia cada medición con el identificador de un landmark del 
    mapa del robot. 
    Se basa en la distancia mínima entre la medida y los landmarks.
    Si la distancia mínima es menor que un umbral se asocia, de lo 
    contrario se asocia a None.
    """
    def do_association(self, measurements):
        associations = []
        
        for measure in measurements:
            dists = []
            for i in range(len(self.xk_k[2:])):
                landmark = self.xk_k[2 + i]
                dist = np.linalg.norm(landmark.p - self.transform_to_global(measure)) 
                dists.append((dist, landmark.id))
        
            min_dist, min_id = min(dists, key=lambda x: x[0]) 
            dists = []

            if min_dist < 0.5:
                associations.append(min_id)
            else:
                associations.append(None)

        return associations
        

# Initial estimates of the position of the error and its covariance matrix
xHat = np.array([0, 0], np.float32)
PHat = np.array([[0,0],[0,0]], np.float32) #la incertidumbre en la posicion inicial del robot es nula

# Object for the (Extended) Kalman filter estimator
kf = KF(xHat, PHat, Q,  R, m)

# Plot the first step of the estimation process and pause
e.plotSim(r, m, kf, True)

measurements = r.measure(m)
associations = [None] * len(measurements)
kf.add_landmarks(measurements, associations)

while r.t < r.tf:

    u = r.action()
    kf.predict(u)
    measurements = r.measure(m)
    associations = kf.do_association(measurements)
    # print("Asociaciones: \n", associations)
#     kf.update(y)

    kf.add_landmarks(measurements, associations) #TODO: SE ESTAN AÑADIENDO TODA LAS LADNMARKS SIEMPRE, AÑADIR SOLO NO ASOCIADAS?
    
    e.plotSim(r, m, kf)
    # time.sleep(3)
