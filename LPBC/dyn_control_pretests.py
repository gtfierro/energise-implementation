import requests
import numpy as np
import time
import datetime as dt
import sys

# manual, Q_control_1, Q_floating_1, Q_floating_2, debug
testID = 'Q_floating_1' 

WtoPerc = 100/7000
VARtoPerc = 100/5000
t0 = time.time()

def enforce_limits(Pcmd,Qcmd):
    if Pcmd != None:
        if Pcmd > 95 or Pcmd < 0.15:
            sys.exit(f'Pcmd violation: {Pcmd}')
    if Qcmd != None:
        if Qcmd > 95 or Qcmd < 0.15:
            sys.exit(f'Qcmd violation: {Qcmd}')
    return


#~~~~~~~~~~~
# INVERTER

if testID == 'manual':

    # inverter values:
    Pcmd_perc_phase = 50*WtoPerc
    Qcmd_perc_phase = 50*VARtoPerc
    inv = 1

    enforce_limits(Pcmd_perc_phase,Qcmd_perc_phase)

    # command = f"http://flexgrid-s1.dhcp.lbl.gov:9090/control?dyn_P_ctrl={Pcmd_perc_phase},inv_id={inv}"
    command = f"http://flexgrid-s1.dhcp.lbl.gov:9090/control?dyn_P_ctrl={Pcmd_perc_phase},dyn_Q_ctrl={Qcmd_perc_phase},inv_id={inv}"
    # command = 'http://flexgrid-s1.dhcp.lbl.gov:9090/status'
    r = requests.get(command) 
    print(f'time to execute: {time.time()-t0}')
    print(r.status_code)
    print('api cmd:', command, dt.datetime.now()) 

if testID == 'Q_control_1':
    Pcmd_perc_phase = 500*WtoPerc
    Qcmd_perc_phase_list = np.array([50,100,250,500,750,1000])*VARtoPerc
    inv_list = [1]
    for itr in range(len(Qcmd_perc_phase_list)):
        Qcmd_perc_phase = Qcmd_perc_phase_list[itr]
        enforce_limits(Pcmd_perc_phase,Qcmd_perc_phase)
        for inv in inv_list:
            if itr != 0:
                time.sleep(15)
            t0 = time.time()
            # command = f"control?dyn_P_ctrl={Pcmd_perc_phase},dyn_Q_ctrl={Qcmd_perc_phase},inv_id={inv}"
            command = f"http://flexgrid-s1.dhcp.lbl.gov:9090/control?dyn_P_ctrl={Pcmd_perc_phase},dyn_Q_ctrl={Qcmd_perc_phase},inv_id={inv}"
            r = requests.get(command)
            print(f'time to execute: {time.time()-t0}')
            print(r.status_code)
            print('api cmd:', command, dt.datetime.now())

if testID == 'Q_floating_1':
    Pcmd_perc_phase_list = np.array([50,100,250,500,750,1000])*WtoPerc
    Qcmd_perc_phase = 50*VARtoPerc
    inv_list = [1]
    for itr in range(len(Pcmd_perc_phase_list)):
        Pcmd_perc_phase = Pcmd_perc_phase_list[itr]
        enforce_limits(Pcmd_perc_phase,Qcmd_perc_phase)
        for inv in inv_list:
            if itr != 0:
                time.sleep(10)
            t0 = time.time()
            # command = f"control?dyn_P_ctrl={Pcmd_perc_phase},dyn_Q_ctrl={Qcmd_perc_phase},inv_id={inv}"
            command = f"http://flexgrid-s1.dhcp.lbl.gov:9090/control?dyn_P_ctrl={Pcmd_perc_phase},dyn_Q_ctrl={Qcmd_perc_phase},inv_id={inv}"
            r = requests.get(command)
            print(f'time to execute: {time.time()-t0}')
            print(r.status_code)
            print('api cmd:', command, dt.datetime.now())

if testID == 'Q_floating_2':
    Pcmd_perc_phase_list = np.array([50,100,250,500,750,1000])* 2 * WtoPerc
    Qcmd_perc_phase = 50*VARtoPerc
    inv_list = [1]
    for itr in range(len(Pcmd_perc_phase_list)):
        Pcmd_perc_phase = Pcmd_perc_phase_list[itr]
        enforce_limits(Pcmd_perc_phase,Qcmd_perc_phase)
        for inv in inv_list:
            if itr != 0:
                time.sleep(15)
            t0 = time.time()
            # command = f"control?dyn_P_ctrl={Pcmd_perc_phase},dyn_Q_ctrl={Qcmd_perc_phase},inv_id={inv}"
            command = f"http://flexgrid-s1.dhcp.lbl.gov:9090/control?dyn_P_ctrl={Pcmd_perc_phase},dyn_Q_ctrl={Qcmd_perc_phase},inv_id={inv}"
            r = requests.get(command)
            print(f'time to execute: {time.time()-t0}')
            print(r.status_code)
            print('api cmd:', command, dt.datetime.now())




if testID == 'debug':

    Pcmd_perc_phase = 50
    Qcmd_perc_phase = None
    enforce_limits(Pcmd_perc_phase,Qcmd_perc_phase)
    print(f'passed -- Pcmd: {Pcmd_perc_phase}, Qcmd: {Qcmd_perc_phase}') 

if testID == 'parallel':
    dummy = None
    # CHANGE TO PARALLEL HTTP COMMANDS 
    # for i in range(1,4):
    #     r = requests.get(command+f',inv_id={i}')
    # print(f'time to execute: {time.time()-t0}')
    # print(r)
    # print('api cmd:', command, dt.datetime.now())

