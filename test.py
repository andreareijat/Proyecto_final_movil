# def association(self, predicted_y, observed_y): 

#     ##FUNCION DE ASOCIACION
#     a = {}
#     threshold = 9.21
#     jacobiana = np.tile([[-1, 0], [0, -1]], (1,1))

#     for i, yi in enumerate(observed_y): 
#         dm_squared = threshold + 1
#         jm = -1
#         for j, yj in enumerate(predicted_y):
#             d_squared = (yi.p-yj).T@np.linalg.inv(jacobiana@self.Pk_k@jacobiana.T + self.R)@(yi.p-yj)
            
#             #Si la distancia es menor a la minima actual
#             if d_squared < dm_squared:
#                 dm_squared = d_squared
#                 jm = j

#         if dm_squared < 9.21: #asocia la medicion con la prediccion de minima distancia si esta por debajo del umbral
#             a[i] = jm
#         else: 
#             a[i] = -1 #no hay asociacion

#     return a

# def calcular_correspondencia(self,p):
#     #funcion de correspondencia que asocia el landmark detectado por el robot a uno del mapa
#     for landmark in self.map: 
#         if landmark == p:
#             x_la, y_la = landmark.p 
#             return x_la, y_la

# def y_residual(self, y):

#     y_gorro=[]

#     for landmark in y:
#         x_la = self.calcular_correspondencia(landmark)
#         y_gorro.append([x_la - self.xk_k])

#     return y_gorro

# """SOLO SE ACTUALIZA LA POSICION DEL ROBOT"""
# def predict(self, u):
#         self.xk_k[:2] = self.A[:2,:2]@self.xk_k[:2] + self.B[:2,:2]@u
#         self.Pk_k = self.A[:2,:2]@self.Pk_k[:2,:2]@self.A[:2,:2].T + self.Q

# """SE ACTUALIZA LA POSICION DEL ROBOT Y LA DE LAS LANDMARKS"""
# def update(self, y):
    
#     robot_position = self.xk_k[:2]
#     predicted_y = [l.p - robot_position for l in y]
#     print("np.shape y: ", np.shape(y), y )
#     print("np.shape predicted y: ", np.shape(predicted_y), predicted_y)
#     y_2 = np.array([(land.p) for land in y])
#     y_residual = y_2-predicted_y
#     a = self.association(predicted_y=predicted_y, observed_y=y)
#     print("HAY {} LANDMARKS".format(len(y)))
    
#     for i, obs in enumerate(y):
#         if a[i] == -1:
            
#             X = ... #pasar del sistema de ref del robot al del mundo

#             #Extension del estado con la nueva posicion
#             self.xk_k = np.concatenate((self.xk_k, X))

#             #Extension de la matriz de covarianzas con la nueva covarianza
#             J_G_inv = ...
#             P_l = self.Pk_k[:2,:2] + J_G_inv@self.R@J_G_inv.T

#             # Extender la matriz de covarianza
#             new_rows = np.zeros((len(self.xk_k), 2))
#             new_cols = np.zeros((2, len(self.xk_k) + 2))

#             self.Pk_k = np.vstack((self.Pk_k, new_rows))
#             self.Pk_k = np.hstack((self.Pk_k, new_cols))

#             self.Pk_k[-2:, -2:] = P_l        
    
#     H = np.tile([[-1, 0], [0, -1]], (len(y), 1))  # Jacobiana de la observación
#     I = np.eye(len(self.xk_k))
    
#     R = np.kron(I,self.R)
#     # Cálculo de la matriz S
#     S = H @ self.Pk_k @ H.T + R

#     # Ganancia de Kalman
#     K = self.Pk_k @ H.T @ np.linalg.inv(S)

#     # Actualización del estado
#     self.xk_k = self.xk_k + K @ y_residual

#     # Actualización de la covarianza
#     self.Pk_k = self.Pk_k - K@S@K.T


import numpy as np


# Definir el primer array de tamaño cuadrado
array1 = np.array([[1, 2, 3, 11], [4, 5, 6, 12], [7, 8, 9, 13], [7, 8, 9, 13]])
print(array1)
# Obtener la dimensión del primer array
import numpy as np
f = array1.shape[0]
c = array1.shape[1]


# Crear un array vacío de tamaño (2*n, 2*n)
result = np.zeros((2+f, 2+c))

# Insertar el primer array en la esquina superior izquierda
result[:f, :c] = array1

# Crear la matriz identidad (n,n) multiplicada por 1000
identity_matrix = np.eye(2) * 1000

# Insertar la matriz identidad en la esquina inferior derecha
result[f:, c:] = identity_matrix

print(result)
