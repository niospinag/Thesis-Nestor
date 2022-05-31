import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import scipy.io as spio
import numpy as np

# save('myData.mat','vhist','zhist','vphist','zphist','hist_pos','T', "ktime", "time_hist")
mat = spio.loadmat('9vehicles/myData.mat', squeeze_me=True)
print(type(mat))
vhist = mat['vhist']  # structures need [()]
vphist = mat['vphist']
hist_pos = mat['hist_pos']
zhist = mat['zhist']
zphist = mat['zphist']
ktime = mat['ktime']
time_hist = mat['time_hist']
T = mat['T']
#  ----------------------------------------------------------------------------

horizon = hist_pos.shape[1]
robots = hist_pos.shape[0]
index_array =['robot ' + str(i+1) for i in range(robots)]
print(vphist.shape)
# print(ktime.shape)
df_vel = pd.DataFrame(vhist[:,:-1], columns = np.arange(1,horizon), index =index_array)
df = pd.DataFrame(time_hist, columns = np.arange(1,horizon), index = index_array)
df_vel.reset_index(drop=True)
# index = pd.Index(index_array, name = 'agent') #pd.Index(np.arange(1,robots+1))
# df = df.set_index(index)
print('df', df)
df_vel.index.names = ['name']
print(df_vel)
# df_merge = df_vel.merge(df, on = index)
# print(df_merge)
# print(type(vphist))
# print(type(zhist))
# print(type(zphist))
# print(time_hist.shape)


# sns.relplot( x = 'Index', y= 'rows' ,data = df, kind='line', )
# plt.show()
