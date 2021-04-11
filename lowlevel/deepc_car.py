## 
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import scipy.io as sio
from casadi import *
from casadi.tools import *

# Random seed:
np.random.seed(1234)

# Customizing Matplotlib:
mpl.rcParams['font.size'] = 18
mpl.rcParams['lines.linewidth'] = 3
mpl.rcParams['axes.grid'] = True
mpl.rcParams['svg.fonttype'] = 'none'
mpl.rcParams['axes.unicode_minus'] = 'true'
mpl.rcParams['axes.labelsize'] = 'large'
mpl.rcParams['legend.fontsize'] = 'large'
mpl.rcParams['xtick.labelsize'] = 'large'
mpl.rcParams['ytick.labelsize'] = 'large'
mpl.rcParams['axes.labelpad'] = 6

# Add subfolder sources to path.
sys.path.append('./sources')
# Import class 'System' from file System.py
from sources.System2 import System
# import System2


# sys_dc = sio.loadmat('./sources/sys_dc.mat')
# A = sys_dc['A_dc']
# B = sys_dc['B_dc']
# C = sys_dc['C']
# D = sys_dc['D']
n_x = 3
n_u = 2

# sys = System2(A,B,C,D)
sys = System(n_x, n_u, dt=1)


def random_u(u0, switch_prob=0.5, u_max=np.pi):
    # Hold the current value with switch_prob chance or switch to new random value.
    u_next = (0.5-np.random.rand(2, 1))*u_max  # New candidate value.
    switch = np.random.rand() >= (1-switch_prob)  # switching? 0 or 1.
    u0 = (1-switch)*u0 + switch*u_next  # Old or new value.
    return u0

#%% simulate and plot system response
sys.reset()

u0 = np.zeros((2, 1))
for k in range(100):
    if k < 50:
        u0 = random_u(u0, u_max=10)
    else:
        u0 = np.zeros((2, 1))
    sys.make_step(u0)


fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
ax[0].plot(sys.time, sys.y)
ax[1].step(sys.time, sys.u)




#%% initialize
T_ini = 4
N = 40

L = T_ini + N

T = 150

n_u = sys.n_u
n_y = sys.n_y



U_L = []
Y_L = []

u0 = np.zeros((2,1))

for k in range(T):
    x0 = np.random.randn(3,1)
    sys.reset(x0)
    
    for k in range(L):
        u0 = random_u(u0)
        sys.make_step(u0)
        
    U_L.append(sys.u.reshape(-1,1))
    Y_L.append(sys.y.reshape(-1,1))
 #%%   
U_L = np.concatenate(U_L,axis=1)
Y_L = np.concatenate(Y_L,axis=1)

assert np.linalg.matrix_rank(U_L) == U_L.shape[0], "not persistantly exciting."

U_Tini, U_N = np.split(U_L, [n_u*T_ini],axis=0)
Y_Tini, Y_N = np.split(Y_L, [n_y*T_ini],axis=0)


#%% trajectories
# Sampled trajectories
n_traj = 8
n_rows = 2

fig, ax = plt.subplots(2, n_traj//n_rows, figsize=(12,6), sharex=True, sharey=True)

for k in range(n_traj):
    i,j = k % n_rows, k//n_rows
    ax[i,j].plot(Y_L[:,k].reshape(-1,n_y))
    
    
    
#%% MULTI STEP PREDICTION MODEL    
    
M = np.concatenate((U_L, Y_Tini))

P = Y_N@np.linalg.pinv(M)


plt.matshow(np.log(np.abs(P)+1e-10))


#%% simulation with the multi step model

y_Tini = sys.y[-T_ini:,:].reshape(-1,1)
u_Tini = sys.u[-T_ini:,:].reshape(-1,1)

u0 = np.zeros((2,1))

for k in range(N):
    u0 = random_u(u0)
    sys.make_step(u0)
    
u_N = sys.u[-N:,:].reshape(-1,1)

b = np.concatenate((u_Tini, u_N, y_Tini),axis=0)

y_true = sys.y[-N:,:]
y_msm = (P@b).reshape(-1,n_y)

    
fig, ax = plt.subplots(3,1, sharex=True, figsize=(12,6))

for k in range(n_y):
    ax[k].plot(y_msm[:,k], label='est')
    ax[k].plot(y_true[:,k], '--', label='True')
    
ax[0].legend()    


print(np.linalg.norm(y_msm-y_true))
    
    
    
#%% making the optimazer

# Create optimization variables:
opt_x = struct_symMX([
    entry('y_N', shape=(n_y), repeat=N),
    entry('u_N', shape=(n_u), repeat=N),
])

# Create parameters of the optimization problem
opt_p = struct_symMX([
    entry('u_Tini', shape=(n_u), repeat=T_ini),
    entry('y_Tini', shape=(n_y), repeat=T_ini),
])

# Create numerical instances of the structures (holding all zeros as entries)
opt_x_num = opt_x(0)
opt_p_num = opt_p(0)
    

# %%
# Create the objective:
obj = 0
for k in range(N):
    obj += sum1(opt_x['y_N',k]**2)+0.1*sum1(opt_x['u_N', k]**2)


# Create the constraints:
b = vertcat(*opt_p['u_Tini'], *opt_x['u_N'], *opt_p['y_Tini'])
y_N = vertcat(*opt_x['y_N'])
cons = P@b-y_N


# Create lower and upper bound structures and set all values to plus/minus infinity.
lbx = opt_x(-np.inf)
ubx = opt_x(np.inf)

# Set only bounds on u_N
lbx['u_N'] = -20
ubx['u_N'] = 20


# Create Optim
nlp = {'x':opt_x, 'f':obj, 'g':cons, 'p':opt_p}
S_msm = nlpsol('S', 'ipopt', nlp)


#%% DEEPC

opt_x_dpc = struct_symMX([
    entry('g', shape=(T)),
    entry('u_N', shape=(n_u), repeat=N),
    entry('y_N', shape=(n_y), repeat=N)
])

opt_p_dpc = struct_symMX([
    entry('u_Tini', shape=(n_u), repeat=T_ini),
    entry('y_Tini', shape=(n_y), repeat=T_ini),
])

opt_x_num_dpc = opt_x_dpc(0)
opt_p_num_dpc = opt_p_dpc(0)

#%% Create the objective:
obj = 0
for k in range(N):
    obj += sum1(opt_x_dpc['y_N',k]**2)+0.1*sum1(opt_x_dpc['u_N', k]**2)

    
# Create the constraints
A = vertcat(U_Tini, U_N, Y_Tini, Y_N)
b = vertcat(*opt_p_dpc['u_Tini'], *opt_x_dpc['u_N'], *opt_p_dpc['y_Tini'], *opt_x_dpc['y_N'])

cons = A@opt_x_dpc['g']-b



# Create lower and upper bound structures and set all values to plus/minus infinity.
lbx_dpc = opt_x_dpc(-np.inf)
ubx_dpc = opt_x_dpc(np.inf)


# Set only bounds on u_N
lbx_dpc['u_N'] = -0.7
ubx_dpc['u_N'] = 0.7

# Create Optim
nlp = {'x':opt_x_dpc, 'f':obj, 'g':cons, 'p':opt_p_dpc}
S_dpc = nlpsol('S', 'ipopt', nlp)


#%% open loop comparison

np.random.seed(12)
sys.reset()

# Excitement
n_exc = 20
u0 = np.zeros((2,1))
for k in range(n_exc):
    u0 = random_u(u0)
    sys.make_step(u0)
    
    
y_Tini = sys.y[-T_ini:,:]
u_Tini = sys.u[-T_ini:,:]


opt_p_num['y_Tini'] = vertsplit(y_Tini)
opt_p_num['u_Tini'] = vertsplit(u_Tini)

opt_p_num_dpc['y_Tini'] = vertsplit(y_Tini)
opt_p_num_dpc['u_Tini'] = vertsplit(u_Tini)


# %% ////////////////////////////SOLVERS///////////


# %MPC BASED ON MULTI-STEP MODEL

r = S_msm(p=opt_p_num,lbx=lbx,ubx=ubx,lbg=0,ubg=0)
# Extract solution
opt_x_num.master = r['x'] 
u_N = horzcat(*opt_x_num['u_N']).full().T
y_N = horzcat(*opt_x_num['y_N']).full().T 


#%% DeePC

r = S_dpc(p=opt_p_num_dpc, lbg=0, ubg=0, lbx=lbx_dpc, ubx=ubx_dpc)
# Extract solution
opt_x_num_dpc.master = r['x'] 
u_N_dpc = horzcat(*opt_x_num_dpc['u_N']).full().T
y_N_dpc = horzcat(*opt_x_num_dpc['y_N']).full().T


#%% comparation

fig, ax = plt.subplots(2,1, figsize=(12,7), sharex=True)

t = np.arange(N)*0.1
y_dpc_lines = ax[0].plot(t,y_N_dpc, linewidth=10, alpha=.5)
ax[0].set_prop_cycle(None)
y_msm_lines = ax[0].plot(t,y_N)
u_dpc_lines = ax[1].step(t,u_N_dpc,linewidth=10, alpha=.5)
ax[1].set_prop_cycle(None)
u_msm_lines = ax[1].step(t,u_N)

plt.sca(ax[0])
ax[0].add_artist(plt.legend(y_dpc_lines,'123', title ='DeePC', loc=1))
ax[0].add_artist(plt.legend(y_msm_lines, '123', title='Proposed', loc=1, bbox_to_anchor=(0.85, 1)))
ax[0].set_ylabel('States \n angle [rad]')


plt.sca(ax[1])
ax[1].add_artist(plt.legend(u_dpc_lines,'12', title ='DeePC', loc=1))
ax[1].add_artist(plt.legend(u_msm_lines, '12', title='Proposed', loc=1, bbox_to_anchor=(0.85, 1)))
ax[1].set_ylabel('Velocities \n angle [rad/s]')



#%% CLOSED LOOP

np.random.seed(12)
sys.reset()

# Excitement
n_exc = 20
u0 = np.zeros((2,1))
for k in range(n_exc):
    u0 = random_u(u0)
    sys.make_step(u0)
    
    
#%%capture
# Control
cost = []
for k in range(60):
    u0 = np.zeros((2,1))
    
    y_Tini = sys.y[-T_ini:,:]
    u_Tini = sys.u[-T_ini:,:] 

    opt_p_num['y_Tini'] = vertsplit(y_Tini)
    opt_p_num['u_Tini'] = vertsplit(u_Tini)
    r = S_dpc(p=opt_p_num, lbg=0, ubg=0, lbx=lbx_dpc, ubx=ubx_dpc)
    opt_x_num_dpc.master = r['x']
    u0 = opt_x_num_dpc['u_N',0].full()
    y0 = sys.make_step(u0)
    
    cost.append(.1*u0.T@u0+y0.T@y0)
    
res_deePC = {'time':sys.time[n_exc:], 'u':sys.u[n_exc:], 'y':sys.y[n_exc:], 'cost':np.concatenate(cost)}


#%% MPC based on multi-step model

np.random.seed(12)
sys.reset(x0=np.zeros((8,1)))

# Excitement
n_exc = 20
u0 = np.zeros((2,1))
for k in range(n_exc):
    u0 = random_u(u0)
    sys.make_step(u0)


#%%capture is faster
# Control
cost = []
for k in range(60):
    u0 = np.zeros((2,1))
    
    y_Tini = sys.y[-T_ini:,:]
    u_Tini = sys.u[-T_ini:,:] 

    opt_p_num['y_Tini'] = vertsplit(y_Tini)
    opt_p_num['u_Tini'] = vertsplit(u_Tini)
    r = S_msm(p=opt_p_num, lbg=0, ubg=0, lbx=lbx, ubx=ubx)
    opt_x_num.master = r['x']
    u0 = opt_x_num['u_N',0].full()
    y0 = sys.make_step(u0)
    
    cost.append(.1*u0.T@u0+y0.T@y0)
    
res_msm = {'time':sys.time[n_exc:], 'u':sys.u[n_exc:], 'y':sys.y[n_exc:], 'cost':np.concatenate(cost)}



#%%///////////////////// comparation/////////////////////////////

fig, ax = plt.subplots(3,2,figsize=(12,8),sharex=True, sharey='row')

t = np.arange(60)*0.1

ax[0,0].set_title('DeePC')
disc_lines = ax[0,0].plot(t,res_deePC['y'])
motor_lines = ax[1,0].step(t,res_deePC['u'],where='post')
ax[2,0].plot(t,res_deePC['cost'])

ax[0,1].set_title('Proposed method')
ax[0,1].plot(t,res_msm['y'][60:])
ax[1,1].step(t,res_msm['u'][60:],where='post')
ax[2,1].plot(t,res_msm['cost'])

ax[0,0].set_ylabel('disc\n angle [rad]')
ax[1,0].set_ylabel('motor\n angle [rad]')
ax[2,0].set_ylabel('cost')

cons_line = ax[1,0].axhline(-0.7,linestyle='--', color='k',linewidth=1)
ax[1,0].axhline(0.7,linestyle='--', color='k',linewidth=1)
ax[1,1].axhline(-0.7,linestyle='--', color='k',linewidth=1)
ax[1,1].axhline(0.7,linestyle='--', color='k',linewidth=1)

ax[0,0].legend(disc_lines,'123', title ='disc', loc=1)
ax[1,0].legend(motor_lines,'12', title ='motor', loc=1)
ax[1,1].legend([cons_line],['bounds'])

ax[2,0].set_xlabel('time [s]')
ax[2,1].set_xlabel('time [s]')

fig.align_ylabels()
fig.align_xlabels()
fig.tight_layout()
plt.show()




print("diferencia del costo",np.sum(res_msm['cost'])-np.sum(res_deePC['cost']))
print("costo msm",np.sum(res_msm['cost']))