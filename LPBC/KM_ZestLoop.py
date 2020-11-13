
import asyncio

from KM_Zestwrapper_CILfunc import *

caution = 2
stepLength = 10 #this is rate in Zestwrapper in seconds
HistLength = 3
# Zeffk_init_mults = [.5, .75, 1.25, 1.5, 2]
Zeffk_init_mults = [.75, 1.25]

# print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ DONE SLEEPING')
# asyncio.sleep(caution)
# print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ DONE SLEEPING')

for i in np.arange(len(Zeffk_init_mults)):
    buildZestimators(Zeffk_init_mults[i],  HistLength)

    loop = asyncio.get_event_loop()
    loop.run_forever()
    print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
    # await asyncio.sleep(caution * stepLength * HistLength)
    asyncio.sleep(caution * stepLength * HistLength)
    print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ DONE SLEEPING')
    loop.stop()
    loop.close()