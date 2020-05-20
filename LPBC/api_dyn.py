import requests
import numpy as np
import time
import datetime as dt

# inverter values:
Pcmd_perc_phase = 10.
inv = 0
    
loop = 0
#loop = 1
    
t0 = time.time()

#~~~~~~~~~~~
# INVERTER
print('here')
command = f"http://131.243.41.48:9090/control?dyn_P_ctrl={Pcmd_perc_phase},inv_id={inv}"
print('here2')
if loop == 0:
    print('here3')
    r = requests.get(command)
    
    print(f'time to execute: {time.time()-t0}')
    print(r)
    print('api cmd:', command, dt.datetime.now())

# CHANGE TO PARALLEL HTTP COMMANDS    
# if loop == 1:
#     for i in range(1,4):
#         r = requests.get(command+f',inv_id={i}')
#     print(f'time to execute: {time.time()-t0}')
#     print(r)
#     print('api cmd:', command, dt.datetime.now())