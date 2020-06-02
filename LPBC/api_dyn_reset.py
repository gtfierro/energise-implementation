import requests
import numpy as np
import time
import datetime as dt
from pymodbus.client.sync import ModbusTcpClient as ModbusClient

IP = '131.243.41.14'
PORT = 504
client = ModbusClient(IP, port=PORT)

# value mapping - 1: [-1, -1], 2: [1, -1], 3: [-1, 1], 4: [1, 1]
# multipliers to inverter values [P, Q] - positive inverter values corresponds to injecting P and Q (value 4)
# FLEXLAB'S QUADRANT CONVENTION 5/22/20 Flexlab set up quadrant convention and will take care of rest into ephasorsim
# Quadrant 1: P consume, Q consume
# Quadrant 2: P inject, Q consume
# Quadrant 3: P consume, Q inject
# Quadrant 4: P inject, Q inject

'''USE THIS SCRIPT BEFORE TESTING TO INITIALIZE INVERTERS'''
# inverter values:
WtoPerc = 100/7000
VARtoPerc = 100/5000

cons_P_offset = 1000*WtoPerc

Pcmd_perc_phase = 50*WtoPerc + cons_P_offset
Qcmd_perc_phase = 50*VARtoPerc
inv_ls = [1,2]

loop = 0
    
t0 = time.time()
#~~~~~~~~~~~
# INVERTER
if loop == 0:

    for inv in inv_ls:
        # command = f"http://flexgrid-s1.dhcp.lbl.gov:9090/control?dyn_P_ctrl={Pcmd_perc_phase},inv_id={inv}"
        command = f"http://flexgrid-s1.dhcp.lbl.gov:9090/control?dyn_P_ctrl={Pcmd_perc_phase},dyn_Q_ctrl={Qcmd_perc_phase},inv_id={inv}"
        # command = 'http://flexgrid-s1.dhcp.lbl.gov:9090/status'
        print(command)
        r = requests.get(command)
        print(f'time to execute: {time.time()-t0}')
        print(r.status_code)
        print('api cmd:', command, dt.datetime.now())


# inverter 1 (phaseA) = 101
# inverter 2 (phaseB)= 102
# inverter 3 (phaseC) = 103

# FLEXLAB'S QUADRANT CONVENTION 5/22/20 Flexlab set up quadrant convention and will take care of rest into ephasorsim
# Quadrant 1: P consume, Q consume
# Quadrant 2: P inject, Q consume
# Quadrant 3: P consume, Q inject
# Quadrant 4: P inject, Q inject

# new (6/1/20)
# 4: +P, -Q (for model: P inj, Q cons)
# 3: -P, -Q (for model: P cons, Q cons)
# 2: +P, +Q (for model: P inj, Q inj)
# 1: -P, +Q (for model: P cons, Q inj)

inv_act_idxs_registers = [101,102,103] #input inverters format: [101], [101,102,103], ... so on...
CIL_offset_registers = [301,302,303,304,305,306]
quadrant = 2 # see above for quadrant convention
CIL_offset = 0
id = 3
t1 = time.time()
try:
    client.connect()
    for idx in inv_ls:  # write quadrant changes to modbus registers
        i = inv_act_idxs_registers[idx-1]
        client.write_registers(i, quadrant, unit=id)
        print('Quadrant change for inv:', i, 'to quadrant', quadrant)
    for idx in inv_ls: # reset P CIL offsets to 0
        i = CIL_offset_registers[(idx-1)*2]
        client.write_registers(i, int(CIL_offset), unit=id)
    for idx in inv_ls: # reset Q CIL offsets to 0
        i = CIL_offset_registers[(idx-1)*2+1]
        client.write_registers(i, int(CIL_offset), unit=id)
    print(f'CIL offsets reset to {CIL_offset}')
except Exception as e:
    print(e)
finally:
    client.close()