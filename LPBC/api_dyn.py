import requests
import numpy as np
import time
import datetime as dt

# inverter values:
WtoPerc = 100/7000
VARtoPerc = 100/5000

Pcmd_perc_phase = 100*WtoPerc
Qcmd_perc_phase = 100*VARtoPerc
inv = 1

loop = 0
    
t0 = time.time()

#~~~~~~~~~~~
# INVERTER

if loop == 0:

    # command = f"http://flexgrid-s1.dhcp.lbl.gov:9090/control?dyn_P_ctrl={Pcmd_perc_phase},inv_id={inv}"
    command = f"http://flexgrid-s1.dhcp.lbl.gov:9090/control?dyn_P_ctrl={Pcmd_perc_phase},dyn_Q_ctrl={Qcmd_perc_phase},inv_id={inv}"
    # command = 'http://flexgrid-s1.dhcp.lbl.gov:9090/status'
    print(command)

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
    for itr in range(0,5):
        t0 = time.time()
        command = f"http://flexgrid-s1.dhcp.lbl.gov:9090/control?dyn_P_ctrl={Pcmd_perc_phase},inv_id={inv}"
        r = requests.get(command)
        print(f'time to execute: {time.time()-t0}')
        print(r.status_code)
        print('api cmd:', command, dt.datetime.now())
        Pcmd_perc_phase += 5
        time.sleep(10)

if loop == 3:
    Pcmd_perc_phase = [5.,10.,8.,5.]
    for itr in range(0,4):
        t0 = time.time()
        command = f"http://flexgrid-s1.dhcp.lbl.gov:9090/control?dyn_P_ctrl={Pcmd_perc_phase[itr]},inv_id={inv}"
        r = requests.get(command)
        print(f'time to execute: {time.time()-t0}')
        print(r.status_code)
        print('api cmd:', command, dt.datetime.now())
        time.sleep(15)