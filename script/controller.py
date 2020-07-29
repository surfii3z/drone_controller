import math
import numpy as np

class Pose3D:
    def __init__(self, rotation, translation):
        self.rotation = rotation
        self.translation = translation

    def inv(self):
        # ref: https://slideplayer.com/slide/5160476/
        return Pose3D(self.rotation.transpose(), -np.dot(self.rotation.transpose(), self.translation))

    def yaw(self):
        # ref: http://planning.cs.uiuc.edu/node103.html 
        # => alpha = atan2(r21/ r11)
        return math.atan2(self.rotation[1, 0], self.rotation[0, 0])

    def __mul__(self, other):
        return Pose3D(np.dot(self.rotation, other.rotation), np.dot(self.rotation, other.translation) + self.translation)

class State:
    def __init__(self): 
        self.position     = np.zeros((2, 1)) # x, y, z
        self.orientation  = np.zeros((3, 1)) # r, p, y
        self.lin_velocity = np.zeros((2, 1)) # vx, vy, vz
        self.ang_velocity = np.zeros((3, 1)) # wx, wy, wz

class PositionController:
    def __init__(self):
        KP_X = 1.0
        KP_Y = 1.0
        KP_Z = 1.0

        KI_X = 0
        KI_Y = 0
        KI_Z = 0

        KD_X = 1.0
        KD_Y = 1.0
        KD_Z = 1.0

        self.Kp_lin = np.array([[KP_X, KP_Y, KP_Z]]).T
        self.Ki_lin = np.array([[KI_X, KI_Y, KI_Z]]).T
        self.Kd_lin = np.array([[KD_X, KD_Y, KD_Z]]).T

    def compute_control_command(self, t, dt, state, state_desired):
        err_lin_w = state_desired.position - state.position
        cur_yaw_w = state.orientation[2]

        u_lin = self.Kp_lin * (err_lin_w) + self.Kd_lin * (state_desired.lin_velocity- state.lin_velocity)

        