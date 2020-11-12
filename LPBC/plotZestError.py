

import pandas as pd
import matplotlib.pyplot as plt

#HERE have to put the file names in manually
#The file names will also be the names in the legend
fileNames = ['λ = .5']

for i in np.arange(len(fileNames)):
    Z_df = pd.read_csv(fileNames[i], index_col=0)
    Z_array = Z_df[['Zth Estimation Error']].to_numpy()
    plt.plot(Z_array, label=fileNames[i]) # this uses the filename for the legend
# plt.title(r'$Z_{Th}$ Estimation Error')
# plt.ylabel('Frobenius Norm Error')
plt.ylabel(r'$Z_{Th}$ Frobenius Norm Error')
plt.xlabel('Timestep')
plt.legend()
plt.show()

for i in np.arange(len(fileNames)):
    Z_df = pd.read_csv(fileNames[i], index_col=0)
    Z_array = Z_df[['Gt']].to_numpy()
    plt.plot(Z_array, label=fileNames[i]) # this uses the filename for the legend
# plt.title('$F$ Magnitude')
plt.ylabel('Frobenius Norm of $F$')
plt.xlabel('Timestep')
plt.legend()
plt.show()

nodeNames = ['Va Magnitude', 'Vb Magnitude', 'Vc Magnitude']]
for i in np.arange(len(nodeNames)):
    Z_df = pd.read_csv(nodeNames[i], index_col=0)
    Z_array = Z_df[['Va Magnitude', 'Vb Magnitude', 'Vc Magnitude']].to_numpy()
    if i == 0:
        plt.plot(Z_array, label='phase A') # this uses the filename for the legend
    elif i == 1:
        plt.plot(Z_array, label='phase B') # this uses the filename for the legend
    else:
        plt.plot(Z_array, label='phase C') # this uses the filename for the legend
# plt.title('Node 675 Voltage Magnitude')
plt.ylabel('Voltage Magnitude [p.u.]')
plt.xlabel('Timestep')
plt.legend()
plt.show()

# ['Zth Estimation Error', 'Gt', 'Va Magnitude', 'Vb Magnitude', 'Vc Magnitude']
