
import pandas as pd
import numpy as np

import sys

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
