import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import scipy.io as spio

# mat = spio.loadmat('myData.mat', squeeze_me=True)

mat = spio.loadmat(filename, squeeze_me=True)

shift_x = -100
scale_x = 1.5

shift_y = -70
scale_y = 25

vhist = mat['vhist']  # structures need [()]
vphist = mat['vphist']
hist_pos = mat['hist_pos']
zhist = mat['zhist']
# zhist.astype=(float)
# zphist = mat['zphist'] * scale_y + shift_y
print('zhist', zhist.shape)
print('hist_pos', hist_pos.shape)

# T = mat['T']
N = hist_pos.shape[0]
horizon = hist_pos.shape[1]