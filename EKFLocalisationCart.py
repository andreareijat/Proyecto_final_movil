import numpy as np
from Simulator import Landmark, Map, Robot, EnvPlot
import time
# Create a map with random landmarks
noLandmarks = 20
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


    def find_landmark_in_map(self, idx):

        for landmark in self.xk_k[2:]:
            if landmark.id == idx: 
                return landmark


    def update(self, measurements, associations):
        print("UPDATE")
        innovation_errors = []

        for i, measure in enumerate(measurements):
            if associations[i] is not None: 
                print("Hay asociacion")

                landmark = self.find_landmark_in_map(associations[i])
                r = self.xk_k[:2] #posicion del robot

                #Calculo de las medidas predichas ŷk = xl - x^k
                predicted_y = landmark.p - r 

                #Calculo del error de innovacion 
                #TODO: corregir desde aqui
                innovation = measure.p - predicted_y
                innovation_errors.append(innovation)

            
        """
        Si por lo que sea en una iteracion no se detecta bien
        un landmark ya conocido se supone que el error de innovacion
        seria la diferencia entre la posicion del robot y la posicion
        predicha del landmark no detectado?
        Otra opcion seria poner una media de los errores de innovacion
        detectados (se escoge esta)
        """

        aux = np.array(innovation_errors)
        print(aux)
        print("aux: ", np.shape(aux))
        if len(innovation_errors) != len(measurements):
            print("Len inn: ", len(innovation_errors))
            print("len land: ", len(self.landmark_ids))
            for i in range(np.abs(len(innovation_errors) - len(measurements))):
                print("Necesito extra")
                innovation = np.mean(aux, axis=0)
                innovation_errors.append(innovation)

        #Jacobiana
        H = np.tile([[-1, 0], [0, -1]], (len(measurements), 1))
        R = np.kron(np.eye(len(measurements)), self.R)
        print("Landmarks: ", len(measurements))
        print("H: ", np.shape(H))
        print("self.Pk_k:", np.shape(self.Pk_k[:2,:2]))
        print("self.R: ", np.shape(R))
        S = H @ self.Pk_k[:2, :2] @ H.T + R
        print("S: ", np.shape(S))
        print("inv S: ", np.shape(np.linalg.inv(S)))
        K = self.Pk_k[:2, :2] @ H.T @ np.linalg.inv(S)
        print("K: ", np.shape(K))
        print("innovation: ", innovation_errors)
        print("innovation: ", np.shape(np.array(innovation_errors).flatten()))
        print("xk_k: ", np.shape(self.xk_k[:2]))
        self.xk_k[:2] = self.xk_k[:2] + K@(np.array(innovation_errors).flatten())
        self.Pk_k[:2,:2] = self.Pk_k[:2,:2] - K@S@K.T
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
                #incertidumbre inicial de las landmarks debe ser inf
                #(definimos un valor alto)
                f = self.Pk_k.shape[0]
                c = self.Pk_k.shape[1]

                result = np.zeros((2+f, 2+c))
                result[:f, :c] = self.Pk_k
                
                identity_matrix = np.eye(2) * 10000
                result[f:, c:] = identity_matrix

                self.Pk_k = result

    
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

        if len(self.xk_k) < 3:
            print("No hay landmarks")
            return [None] * len(measurements)
        
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
xHat = np.array([r.p[0], r.p[1]], np.float32)
PHat = np.array([[0,0],[0,0]], np.float32) #la incertidumbre en la posicion inicial del robot es nula

# Object for the (Extended) Kalman filter estimator
kf = KF(xHat, PHat, Q,  0.01*R, m)

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
    print("Asociaciones: \n", associations)
    kf.update(measurements, associations)

    kf.add_landmarks(measurements, associations)
    
    e.plotSim(r, m, kf)
    # time.sleep(2)
