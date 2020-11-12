

from KM_Zestwrapper_CILfunc import *

caution = 1.1
stepLength = 10 #this is rate in Zestwrapper in seconds
HistLength = 3
# Zeffk_init_mults = [.5, .75, 1.25, 1.5, 2]
Zeffk_init_mults = [.75, 1.25]
for i in np.arange(len(Zeffk_init_mults)):
    buildZestimators(Zeffk_init_mults[i],  HistLength)

    loop.run_forever()
    # await asyncio.sleep(caution * stepLength * HistLength)
    asyncio.sleep(caution * stepLength * HistLength)
    loop.stop()
    loop.close()
