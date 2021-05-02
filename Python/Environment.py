# autonomous vehicles

from gurobipy import *
import numpy as np
import time
import sys
# import sources.deepc
from vehicle import Car
import matplotlib.pyplot as plt

# %%

nx = 1  # Number of agents
nu = 1  # Number of inputs
nv = 2  # numero de vehiculos sin el agente no cooperativo
# MPC data
Q = 1  # * np.identity(1)
R = 10  # * np.identity(1)
N = 7  # horizon
dt = 0.1  # [s]
Ds = 7  # Safety distance [m]
Dl = 25  # lateral distance
V_max = 80
A_max = 30
L = 6  # number of lanes
Mmax = L - 1
mmin = -L + 1
p_max = 1

# %------condiciones iniciales----------
vel = np.array([20, 20])  # velociodad inicial
Vdes = np.array([30, 50])  # velocidad deseada

zel = np.array([5, 4])  # carril inicial
Zdes = np.array([1, 1])  # carril deseado

acel = np.array([0, 0])
# %---distancia inicial de cada agente
d1i = np.array([-50])
i = 0
zel2 = zel  # same dimentions
LR2 = 1
LR1 = 1

# %%
# create the model
vh1 = Car(vel[0], zel[0], Vdes[0], Zdes[0], N, name="vehiculo_1")
vh2 = Car(vel[1], zel[1], Vdes[1], Zdes[1], N, name="vehiculo_2")

# %%


vh1.create_model()
vh2.create_model()

vh1.m.update()
vh2.m.update()

# %%

# %%capture --no-stderr
t = time.time()
sim_tim = 20
for k in range(sim_tim):
    vh1.update_cnt(zel[0], vel[0], Zdes[0], Vdes[0], vel[1], zel[1], d1i[0])
    vh1.m.update()
    vh1.m.optimize()
    vh2.update_cnt(zel[1], vel[1], Zdes[1], Vdes[1], vel[0], zel[0], -d1i[0])
    vh2.m.update()
    vh2.m.optimize()

    # get optimal solution
    acel[0], zel[0] = vh1.get_var(N)
    acel[1], zel[1] = vh2.get_var(N)

    vel[0], zel[0] = vh1.make_step(acel[0], zel[0])
    vel[1], zel[1] = vh2.make_step(acel[1], zel[1])

    d1i = d1i + dt * (vel[1:] - vel[0])

elapsed = time.time() - t

# %%

print("Elapsed time: {}[s]".format(round(elapsed, 3)))
print("the objective function is :" + str(round(vh1.m.ObjVal, 2)))
for v in vh1.m.getVars():
    print(str(v.VarName), v.x)


# %%

hist_pred_z1 = np.array(vh1.hist_zp1)
hist_pred_z2 = np.array(vh2.hist_zp1)
hist_pred_v1 = np.array(vh1.hist_vp1)
hist_pred_v2 = np.array(vh2.hist_vp1)
hist_time = np.arange(0, sim_tim)
hist_pred_v2.shape

# %%

# plot the data
# fig = plt.figure()
fig, (ax1, ax2) = plt.subplots(2, 1)
# ax = fig.add_subplot(2, 1, 1)
ax1.plot(hist_time, hist_pred_z1[:, 0], marker='o', color='tab:blue')
ax1.plot(hist_time, hist_pred_z2[:, 0], marker='o', color='tab:orange')
ax1.set_xlabel('iteration')
ax1.set_ylabel('lane')
ax1.grid(True)

ax2.plot(hist_time, hist_pred_v1[:, 0], marker='o', color='tab:blue')
ax2.plot(hist_time, hist_pred_v2[:, 0], marker='o', color='tab:orange')
ax2.set_xlabel('iteration')
ax2.set_ylabel('velocity')
ax2.grid(True)

# %%
