

from KM_Zestwrapper_CILfunc import *

HistLength = 3
# Zeffk_init_mults = [.5, .75, 1.25, 1.5, 2]
Zeffk_init_mults = [.75, 1.25]
for i in len(Zeffk_init_mults):
    buildZestimators(Zeffk_init_mults[i],  HistLength)
