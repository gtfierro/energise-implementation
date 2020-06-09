
import pandas as pd
import numpy as np

import sys


nphases = 3
print(np.zeros(2*nphases))
u = np.asmatrix(np.zeros(2*nphases))

print(u)
print(np.shape(u))

sys.exit()

e = np.asarray([1,np.NaN,3])
if any(np.isnan(e)):
    print('here')

sys.exit()

e = np.asarray([1,2,3])
g = np.asarray([1,2,2])
if all(abs(e-g) < .1):
    print('here')
sys.exit()

f = ['a','b','c']

print(list(enumerate(f)))
for i,let in reversed(list(enumerate(f))):
    print(i)
    print(let)
# for i in reversed(np.arange(len(f))):
for i in reversed(range(len(f))):
    print(i)
    print(f[i])

sys.exit()

testcase = '13bal'
busId = '675'

Zeffkpath = 'networkImpedanceModels/Zeffks/' + str(testcase) + '/PU' + '/Zeffk_bus' + str(busId) + '.csv'
if testcase == 'manual': #HERE for debugging, assumes 13bal is used
    # Zeffkpath = 'networkImpedanceModels/Zeffks/' + '13bal' + '/notPU' + '/Zeffk_bus' + str(busId) + '.csv' #alternative
    Zeffkpath = 'networkImpedanceModels/Zeffks/' + '13bal' + '/PU' + '/Zeffk_bus' + str(busId) + '.csv'
Zeffk_df = pd.read_csv(Zeffkpath, index_col=0) #index_col=0 bc of how Im saving the df (should have done index = false)
Zeffk_df = Zeffk_df.apply(lambda col: col.apply(lambda val: complex(val.strip('()')))) #bc data is complex
Zeffk_init = np.asmatrix(Zeffk_df.values)

print(Zeffk_init)

sys.exit()

nphases = 3
Vang = np.asarray([np.NaN]*nphases)
Vang[0] = 1
Vang[1] = 1
Vang[2] = 1
print(Vang)
if any(np.isnan(Vang)):
    print('here')
print('hhere')

sys.exit()


testcase = '13bal'
busId = '611'

Zskpath = 'Zsks/Zsks_pu_' + str(testcase) + '/Zsk_bus' + str(busId) + '.csv'
Zsk_df = pd.read_csv(Zskpath, index_col=0)
Zsk_df = Zsk_df.apply(lambda col: col.apply(lambda val: complex(val.strip('()'))))
Zskinit = np.asmatrix(Zsk_df.to_numpy())

print(Zskinit)
