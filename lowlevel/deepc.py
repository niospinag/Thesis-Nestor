# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'
# %%
from IPython import get_ipython

# %% [markdown]
# # Deterministic case: DeePC vs. MPC based on multi-step model 
# 
# This notebook is used to create the results for **Figure 2** in our paper.

# %%
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import scipy.io as sio
from casadi import *
from casadi.tools import *


# %%
# Random seed:
np.random.seed(1234)


# %%
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

# %% [markdown]
# ##  System description
# 
# 
# We investigate a simple LTI system in the form of:
# \begin{align}
# x_{k+1} &= Ax_k + Bu_k\\
# y_k &= Cx_k+Du_k,
# \end{align}
# 
# with $x \in \mathbb{R}^n$ (states), $u\in \mathbb{R}^m$ (inputs), $y\in \mathbb{R}^p$ (measurements) and system matrices $A \in \mathbb{R}^{n\times n}$, $B \in \mathbb{R}^{n\times m}$, $C \in \mathbb{R}^{p\times n}$ and $D \in \mathbb{R}^{p\times m}$.
# 
# The investigated system, displayed below, is a triple-mass-spring system (rotating discs) with two stepper motors ($m=2$) attached to the outermost discs via additional springs. These disc angles are the measured output of the system ($p=3$). The system has a total of $n=8$ states.
# %% [markdown]
# <img src="sources/triple_mass_spring.pdf" width=40%/>
# %% [markdown]
# To simulate the system, we created a very simple helper class, that keeps the current state and stores the past sequences of inputs, states and outputs.
# 
# We import the system class with:

# %%
import sys
# Add subfolder sources to path.
sys.path.append('./sources')
# Import class 'System' from file System.py
from System2 import System

# %% [markdown]
# Import system matrices, $A,B,C,D$ and create instance of ``System``:

# %%
sys_dc = sio.loadmat('./sources/sys_dc.mat')
A = sys_dc['A_dc']
B = sys_dc['B_dc']
C = sys_dc['C']
D = sys_dc['D']

n_x = 3
n_u = 2
sys = System(n_x, n_u, dt=1)

# %% [markdown]
# ## Investigating the System
# %% [markdown]
# We create a helper function that creates an input signal reminiscent of PRBS (but with varying amplitude). 

# %%
def random_u(u0, switch_prob=0.5, u_max=20):
    # Hold the current value with switch_prob chance or switch to new random value.
    u_next = (0.5-np.random.rand(2,1))*u_max # New candidate value.
    switch = np.random.rand() >= (1-switch_prob) # switching? 0 or 1.
    u0 = (1-switch)*u0 + switch*u_next # Old or new value.
    return u0

# %% [markdown]
# Simulate and plot system response.

# %%
sys.reset()

u0 = np.zeros((2,1))
for k in range(100): #100
    if k<50:
        u0 = random_u(u0)
    else:
        u0 = np.zeros((2,1))
    sys.make_step(u0)


fig, ax = plt.subplots(2,1,figsize=(10,6), sharex = True)
ax[0].plot(sys.time,sys.y)
ax[1].step(sys.time,sys.u)

# %% [markdown]
# # DeePC vs. MPC based on multi-step model
# %% [markdown]
# ## Data collection
# 
# Both methods require a measured system response for persistantly exciting input data, depending on the parameters:
# $T_{\text{ini}}$ and $N$.
# 
# The total number of measured sequences depends on these parameters and must exceed the thresholds given by Assumption 1 and 2 in the paper. 
# 
# In the particular example with $T_{\text{ini}}=4$ and $N=40$, we have that $T\geq  96$ according to Assumption 1 and $T\geq  100$ according to Assumption 2.
# 
# We choose $T\geq  150$.

# %%
T_ini = 4
N = 40

L = T_ini + N

T = 200

n_u = sys.n_u
n_y = sys.n_y

# %% [markdown]
# Create matrices $U_L$ and $Y_L$ according to (2) in the paper. Note that we construct the data matrices which are neither Hankel nor Page matrices. Instead we "measure" independent sequences each with **random initial state**.

# %%
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
    
U_L = np.concatenate(U_L,axis=1)
Y_L = np.concatenate(Y_L,axis=1)

assert np.linalg.matrix_rank(U_L) == U_L.shape[0], "not persistantly exciting."

# %% [markdown]
# According to (3) in the paper, we divide the input/output data in two parts such that:
# 
# \begin{equation}
# U_L = \left[
# \begin{array}{c}
# U_{T_\text{ini}}\\
# U_{N}
# \end{array}
# \right],\ 
# Y_L = \left[
# \begin{array}{c}
# Y_{T_\text{ini}}\\
# Y_{N}
# \end{array}
# \right].
# \end{equation}

# %%
U_Tini, U_N = np.split(U_L, [n_u*T_ini],axis=0)
Y_Tini, Y_N = np.split(Y_L, [n_y*T_ini],axis=0)

# %% [markdown]
# Here are some of the sampled trajectories:

# %%
n_traj = 8
n_rows = 2

fig, ax = plt.subplots(2, n_traj//n_rows, figsize=(12,6), sharex=True, sharey=True)

for k in range(n_traj):
    i,j = k % n_rows, k//n_rows
    ax[i,j].plot(Y_L[:,k].reshape(-1,n_y))

# %% [markdown]
# ## Multi-step prediction model:
# %% [markdown]
# For the multi-step model we identify $P^*$ from the least-squares problem:
# 
# \begin{equation}
# %
# 	 P^* = \arg\ \min_{ P} \ \| P
# 	\underbrace{
# 	\left[\begin{array}{c}
# 	U_{T_\text{ini}}\\
# 	U_N\\
# 	Y_{T_\text{ini}}
# 	\end{array}
# 	\right]}_M
# 	- 
# 	Y_N
# 	\|_2^2.
# \end{equation}
# 
# with the Moore-Penrose inverse, such that:
# 
# \begin{equation}
#  P^* = Y_N M^\dagger,
# \end{equation}

# %%
M = np.concatenate((U_L, Y_Tini))

P = Y_N@np.linalg.pinv(M)

# %% [markdown]
# Let's see how $P^*$ looks like:

# %%
plt.matshow(np.log(np.abs(P)+1e-10))

# %% [markdown]
# We expected this triangular shape, because there is no relationship between outputs and their respective future inputs. Also the block to the right shows that the entire sequence depends on the initial measurements.
# %% [markdown]
# ### Simulation with the multi-step model
# 
# To validate the multi-step model, we simulate $N$ steps into the future for given initial inputs/outputs ($y_{T_{\text{ini}}},\  u_{T_{\text{ini}}}$) and a random future input-sequence ($u_N$).

# %%
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


# %%
fig, ax = plt.subplots(3,1, sharex=True, figsize=(12,6))

for k in range(n_y):
    ax[k].plot(y_msm[:,k], label='est')
    ax[k].plot(y_true[:,k], '--', label='True')
    
ax[0].legend()

# %% [markdown]
# To quantify the prediction accuracy, we compute:

# %%
np.linalg.norm(y_msm-y_true)

# %% [markdown]
# which shows that both results are nearly identical.
# %% [markdown]
# ### MPC based on multi-step model: Building the optimizer
# %% [markdown]
# We want to implement the following problem:
# 
# \begin{equation}
# 	\begin{aligned}
# 		\min_{u_N,y_N}\quad &\sum_{k=1}^N \left(\|y_k\|^2_{Q}+\|u_k\|^2_{R}\right)\\
# 		\text{s.t.}\quad & 
#         y_N =  P^*
#         \left[\begin{array}{c}
#         u_{T_\text{ini}}\\
#         u_{N}\\
#         y_{T_\text{ini}}\\
#         \end{array}\right],
#         \\
# 		&u_k \in \mathbb{U},\ \forall k\in \{1,\dots,N\},	\\
# 		&y_k \in \mathbb{Y},\ \forall k\in \{1,\dots,N\}.	\\
# 	\end{aligned}
# \end{equation}
# 
# The results in the paper are obtained with $\mathbb{U}=[-0.7,0.7]$ for both motors.
# 
# We build the optimizer using [CasADi](https://web.casadi.org) and profit from their ``struct_symSX`` class, which allows to create symbolic variables within a structure for easier indexing. This [tutorial](http://casadi.sourceforge.net/v2.0.0/tutorials/tools/structure.pdf) explains the use of structures for CasADi in Python.

# %%
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

# %% [markdown]
# ## DeePC
# 
# ### Building the optimizer
# 
# We now proceed to implement DeePC using the same tools. The optimization problem is given as:
# 
# \begin{equation}
# 	\begin{aligned}
# 	\min_{g,u_N, y_N}\quad &f(g,u_N, y_N)\\
# 	\text{s.t.}\quad & 
#     \left[\begin{array}{c}
# 	U_{T_\text{ini}}\\
# 	U_{N}\\
# 	Y_{T_\text{ini}}\\
# 	Y_{N}
# 	\end{array}\right] g 
# 	= 
# 	\left[\begin{array}{c}
# 	u_{T_\text{ini}}\\
# 	u_{N}\\
# 	y_{T_\text{ini}}\\
# 	y_{N}
# 	\end{array}\right],
#     \\
# 	&u_k \in \mathbb{U}\ \forall k\in\{1,\dots,N\},	\\
# 	&y_k \in \mathbb{Y}\ \forall k\in\{1,\dots,N\}, \\
# 	\end{aligned}
# \end{equation}

# %%
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


# %%
# Create the objective:
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
lbx_dpc['u_N'] = -20
ubx_dpc['u_N'] = 20

# Create Optim
nlp = {'x':opt_x_dpc, 'f':obj, 'g':cons, 'p':opt_p_dpc}
S_dpc = nlpsol('S', 'ipopt', nlp)

# %% [markdown]
# # Open-loop comparison
# 
# We start with a comparison of the open-loop prediction for some arbitrary intial condition (not at rest).

# %%
np.random.seed(12)
sys.reset()

# Excitement
n_exc = 20
u0 = np.zeros((2,1))
for k in range(n_exc):
    u0 = random_u(u0)
    sys.make_step(u0)


# %%
y_Tini = sys.y[-T_ini:,:]
u_Tini = sys.u[-T_ini:,:]

# %% [markdown]
# Assign these initial conditions to the parameter structures for both optimizers

# %%
opt_p_num['y_Tini'] = vertsplit(y_Tini)
opt_p_num['u_Tini'] = vertsplit(u_Tini)

opt_p_num_dpc['y_Tini'] = vertsplit(y_Tini)
opt_p_num_dpc['u_Tini'] = vertsplit(u_Tini)

# %% [markdown]
# ## Solve MPC based on multi-step model

# %%
r = S_msm(p=opt_p_num,lbx=lbx,ubx=ubx,lbg=0,ubg=0)
# Extract solution
opt_x_num.master = r['x'] 
u_N = horzcat(*opt_x_num['u_N']).full().T
y_N = horzcat(*opt_x_num['y_N']).full().T


# %%
r = S_dpc(p=opt_p_num_dpc, lbg=0, ubg=0, lbx=lbx_dpc, ubx=ubx_dpc)
# Extract solution
opt_x_num_dpc.master = r['x'] 
u_N_dpc = horzcat(*opt_x_num_dpc['u_N']).full().T
y_N_dpc = horzcat(*opt_x_num_dpc['y_N']).full().T

# %% [markdown]
# ## Comparison of open-loop prediction

# %%
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
ax[0].set_ylabel('disc \n angle [rad]')


plt.sca(ax[1])
ax[1].add_artist(plt.legend(u_dpc_lines,'12', title ='DeePC', loc=1))
ax[1].add_artist(plt.legend(u_msm_lines, '12', title='Proposed', loc=1, bbox_to_anchor=(0.85, 1)))
ax[1].set_ylabel('motor \n angle [rad]')

# %% [markdown]
# We can also compare the norm of the difference of the obtained solutions:

# %%
np.linalg.norm(y_N_dpc-y_N)


# %%
np.linalg.norm(u_N_dpc-u_N)

# %% [markdown]
# which shows that they are identical.
# %% [markdown]
# # Closed-loop comparison:
# %% [markdown]
# ## DeePC
# 
# We start by resetting the system and applying a (pseudo) random input sequence. 
# By fixing the random seed to 12 (why not 12?), the initilization is repeatable. 

# %%
np.random.seed(12)
sys.reset()

# Excitement
n_exc = 20
u0 = np.zeros((2,1))
for k in range(n_exc):
    u0 = random_u(u0)
    sys.make_step(u0)

# %% [markdown]
# We then proceed with the closed-loop control experiment and store the results in the ``res_deePC`` dict.
# The output of this code cell is surpressed. with ``%%capture``. 

# %%
get_ipython().run_cell_magic('capture', '', "# Control\ncost = []\nfor k in range(60):\n    u0 = np.zeros((2,1))\n    \n    y_Tini = sys.y[-T_ini:,:]\n    u_Tini = sys.u[-T_ini:,:] \n\n    opt_p_num['y_Tini'] = vertsplit(y_Tini)\n    opt_p_num['u_Tini'] = vertsplit(u_Tini)\n    r = S_dpc(p=opt_p_num, lbg=0, ubg=0, lbx=lbx_dpc, ubx=ubx_dpc)\n    opt_x_num_dpc.master = r['x']\n    u0 = opt_x_num_dpc['u_N',0].full()\n    y0 = sys.make_step(u0)\n    \n    cost.append(.1*u0.T@u0+y0.T@y0)\n    \nres_deePC = {'time':sys.time[n_exc:], 'u':sys.u[n_exc:], 'y':sys.y[n_exc:], 'cost':np.concatenate(cost)}")

# %% [markdown]
# ## MPC based on multi-step model
# 
# Similarly as above, we first initilize the sytem.

# %%
np.random.seed(12)
sys.reset(x0=np.zeros((8,1)))

# Excitement
n_exc = 20
u0 = np.zeros((2,1))
for k in range(n_exc):
    u0 = random_u(u0)
    sys.make_step(u0)

# %% [markdown]
# Control-loop for MPC based on multi-step model:

# %%
get_ipython().run_cell_magic('capture', '', "# Control\ncost = []\nfor k in range(60):\n    u0 = np.zeros((2,1))\n    \n    y_Tini = sys.y[-T_ini:,:]\n    u_Tini = sys.u[-T_ini:,:] \n\n    opt_p_num['y_Tini'] = vertsplit(y_Tini)\n    opt_p_num['u_Tini'] = vertsplit(u_Tini)\n    r = S_msm(p=opt_p_num, lbg=0, ubg=0, lbx=lbx, ubx=ubx)\n    opt_x_num.master = r['x']\n    u0 = opt_x_num['u_N',0].full()\n    y0 = sys.make_step(u0)\n    \n    cost.append(.1*u0.T@u0+y0.T@y0)\n    \nres_msm = {'time':sys.time[n_exc:], 'u':sys.u[n_exc:], 'y':sys.y[n_exc:], 'cost':np.concatenate(cost)}")

# %% [markdown]
# **Figure 2** in paper.
# 
# Closed-loop trajectories of the mass-spring-system. Regulation after some initial excitation. Comparison of DeePC  and our proposed method with deterministic data.

# %%
fig, ax = plt.subplots(3,2,figsize=(12,8),sharex=True, sharey='row')

t = np.arange(60)*0.1

ax[0,0].set_title('DeePC')
disc_lines = ax[0,0].plot(t,res_deePC['y'])
motor_lines = ax[1,0].step(t,res_deePC['u'],where='post')
ax[2,0].plot(t,res_deePC['cost'])

ax[0,1].set_title('Proposed method')
ax[0,1].plot(t,res_msm['y'])
ax[1,1].step(t,res_msm['u'],where='post')
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

# %% [markdown]
# To quantify the similarity we look at the difference of the cost:

# %%
np.sum(res_msm['cost'])-np.sum(res_deePC['cost'])

# %% [markdown]
# The overall cost in this scenario is:

# %%
np.sum(res_msm['cost'])


