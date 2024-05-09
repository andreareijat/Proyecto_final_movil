import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms

# Landmark class to store a landmark position and its ID
# This class overloads the operator == (__eq__) to be able
# to compare Landmark objects according to their ID
class Landmark:
    def __init__(self, p, id):
        self.p = p
        self.id = id

    def __add__(self, p1):
        return self.p + p1

    def __sub__(self, p1):
        return self.p - p1

    def __eq__(self, id):
        if isinstance(id, Landmark):
            return self.id == id.id
        return self.id == id

# Map class derived from a list to hold the Landmark  objects
class Map(list):
    def __init__(self):
        # self.ll = []
        super().__init__()
        
# Class Robot implements an integrator robot with a sensor that can
# perceive the landmarks in the environment within a given range. The
# matrices Q and R are the (diagonal) state and sensor noise
# covariance matrices of the robot, while the parameter type tells the
# sort of measurements of the landmards the robot provides, which can
# be 'xy' for the Cartesian coordinates of the landmarks (relative to
# the robot) or 'rb' (Range & Bearing) for the range and bearing of
# the landmarks
class Robot:
    def __init__(self, pose, Q, R, range=2.5, type='xy'):
        self.p = pose
        self.range = range
        self.type = type
        self.dt = 0.25;
        self.Q = Q * self.dt
        self.R = R / self.dt
        self.objs = []      # obs is a list containing the observed
                            # landmarks at each time step
        self.t = 0
        self.v = 0.5
        self.ti = 8
        self.tf = 5 * self.ti
        
    # This method populates the field 'obs' (observed landmarks) given
    # the map 'm' passed as an argument. The vectors contained in the
    # list will be Cartesian coordiantes or range and bearing
    # depending on the type field of the robot ('xy' or 'rb')
    def measure(self, m):
        self.objs = list()
        # Loop for every landmark in the map
        for l in m:
            p = self.p
            # If the distance from the robot to the landmark is
            # smaller than the robot perceptual range, include the
            # landmark in the list of observed landmarks
            if np.linalg.norm(p - l.p) <= self.range:
                if self.type == 'rb':
                    p0  = np.array([np.linalg.norm(p - l.p),
                                    np.arctan2(l.p[1] - p[1],
                                               l.p[0] - p[0])], np.float32)
                else:
                    p0 = l.p - self.p

                # Add noise to the measurement
                p0 += np.array([np.random.normal(0, self.R[0,0]),
                                np.random.normal(0, self.R[1,1])], np.float32)
                self.objs.append(Landmark(p0, l.id))
                
        return self.objs

    # This method sets some speed of the integrator robot to make it
    # move in a square trajectory.
    def action(self):
        v = np.array([0,0], np.float32)
        istep = self.t / self.ti
        if istep < 1.0:
            v[0] = 0
            v[1] = self.v
        elif istep < 2.0:
            v[0] = -self.v
            v[1] = 0
        elif istep < 3.0:
            v[0] = 0.0
            v[1] = -self.v
        elif istep < 4.0:
            v[0] = self.v
            v[1] = 0
        else:
            v[0] = 0
            v[0] = 0
        self.p += self.dt * v
        self.p += np.array([np.random.normal(0, self.Q[0][0]),
                            np.random.normal(0, self.Q[1][1])],np.float32)
        self.t += self.dt
        return v
                    
# Class to plot the environment, the robot and the estimate from the
# Kalman filter
class EnvPlot:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.gca()
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        pass

    def plotSim(self, r, m, kf, pause=False):
        # Clear the plot
        self.ax.cla()
        # Draw the map, i.e. landmarks
        for l in m:
            self.ax.plot(l.p[0], l.p[1], marker='*', color='k')

        # Draw the perception range of the robot
        c = plt.Circle((r.p[0], r.p[1]), r.range, color='blue', fill=False,
                       linestyle='--')
        self.ax.add_patch(c)
        self.ax.plot(r.p[0], r.p[1], '+', color='b')
        # Draw the objects perceived by the robot
        for l in r.objs:
            if r.type == 'xy':
                p = r.p + l.p
            else:
                p = r.p + np.array([l.p[0] * np.cos(l.p[1]),
                                    l.p[0] * np.sin(l.p[1])], dtype=np.float32)
            self.ax.plot(p[0], p[1], marker='*', color='b')
            
        # Draw the estimate of the Kalman filter
        f = 3.0;
        S = kf.Pk_k[0:3,0:3]
        pearson = S[0, 1]/np.sqrt(S[0, 0] * S[1, 1])
        # Using a special case to obtain the eigenvalues of this
        # two-dimensionl dataset.
        ell_radius_x = np.sqrt(1 + pearson)
        ell_radius_y = np.sqrt(1 - pearson)
        ellipse = Ellipse((0, 0), width=ell_radius_x * 2,
                          height=ell_radius_y * 2, edgecolor='red',
                          fc='none')

        scale_x = np.sqrt(S[0, 0]) * f
        scale_y = np.sqrt(S[1, 1]) * f

        transf = transforms.Affine2D() \
                           .rotate_deg(45) \
                           .scale(scale_x, scale_y) \
                           .translate(kf.xk_k[0], kf.xk_k[1])

        ellipse.set_transform(transf + self.ax.transData)
        self.ax.add_patch(ellipse)
        self.ax.plot(kf.xk_k[0], kf.xk_k[1], marker='+', color='r')
        self.ax.set_xlim(-5.1, 5.1)
        self.ax.set_ylim(-5.1, 5.1)

        
        plt.draw()
        plt.pause(r.dt)
        if pause:
            plt.waitforbuttonpress(0)
