import requests
import numpy as np
import time
import datetime as dt

# inverter values:
Pcmd_perc_phase = 5.
inv = 1
    
loop = 0
#loop = 1
loop = 2
    
t0 = time.time()

#~~~~~~~~~~~
# INVERTER

# command = f"http://flexgrid-s1.dhcp.lbl.gov:9090/control?dyn_P_ctrl={Pcmd_perc_phase},inv_id={inv}"
# command = 'http://flexgrid-s1.dhcp.lbl.gov:9090/status'
# print(command)

if loop == 0:

    r = requests.get(command)
    
    print(f'time to execute: {time.time()-t0}')
    print(r.status_code)
    print('api cmd:', command, dt.datetime.now())

# CHANGE TO PARALLEL HTTP COMMANDS    
# if loop == 1:
#     for i in range(1,4):
#         r = requests.get(command+f',inv_id={i}')
#     print(f'time to execute: {time.time()-t0}')
#     print(r)
#     print('api cmd:', command, dt.datetime.now())

if loop == 2:
    Pcmd_perc_phase = 5.
    for itr in range(0,10):
        t0 = time.time()
        command = f"http://flexgrid-s1.dhcp.lbl.gov:9090/control?dyn_P_ctrl={Pcmd_perc_phase},inv_id={inv}"
        r = requests.get(command)
        print(f'time to execute: {time.time()-t0}')
        print(r.status_code)
        print('api cmd:', command, dt.datetime.now())
        Pcmd_perc_phase += 5
        time.sleep(2)

