#!/usr/bin/python3
import numpy as np
from gurobipy import *

# class vehiculo:
#     def __init__(self):
#         self.Q = 1
#         self.R = 10
#         self.N = 3
#         self.T = 1
#         self.Ds = 15 # safety distance
#         self.Dl = 25
#         self.v_max = 80
#         self.a_max = 30
#         self.l= 6 #Number of lines
#         self.m_max = self.l - 1
#         self.m_min = -self.l  + 1
#
#     def make_step(self, a, z):

nx = 1  # Number of agents
nu = 1  # Number of inputs
nv = 2  # Number of vehicles
# MPC data
Q = 1 * eye(1)
R = 10 * eye(1)
N = 3  # horizon
T = 0.1  # [s]
Ds = 7  # Safety distance [m]
Dl = 25  # lateral distance
V_max = 80
A_max = 30
L = 6  # number of lanes
Mmax = L - 1
mmin = -L + 1
p_max = 1

m = Model("vehiculo")
